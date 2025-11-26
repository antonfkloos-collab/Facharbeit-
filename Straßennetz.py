# ---------------------------------------------------
# 1. Kartendaten laden (Siegen + Eiserfeld)
# ---------------------------------------------------
import osmnx as ox
import heapq
import folium
from folium.plugins import MarkerCluster, HeatMap
import networkx as nx
import os
from math import radians, cos, sin, asin, sqrt
import numpy as np
from sklearn.linear_model import RidgeCV
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
import re
# -*- coding: utf-8 -*-
# Facharbeit: Navigation mit Unfalldaten (Autonomes Fahren)


def haversine_meters(lat1, lon1, lat2, lon2):
    # return distance in meters between two lat/lon points
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    return 6371000 * c


def load_graph(place_name="Siegen, Germany"):
    try:
        G = ox.graph_from_place(place_name, network_type="drive")
        return G
    except Exception as e:
        raise RuntimeError(f"Graph konnte nicht geladen werden: {e}")


G = load_graph("Siegen, Germany")

# Start- und Zielpunkte (lat, lon)
try:
    start_coords = ox.geocode("Gesamtschule Eiserfeld, Siegen, Germany")
except Exception:
    start_coords = (50.8542, 8.0248)

try:
    ziel_coords = ox.geocode("Siegen ZOB, Siegen, Germany")
except Exception:
    # fallback to city center coordinates
    ziel_coords = (50.8748, 8.0243)

# nearest_nodes erwartet (G, X, Y) = (G, lon, lat) for the point lookup
start = ox.distance.nearest_nodes(G, start_coords[1], start_coords[0])
ziel = ox.distance.nearest_nodes(G, ziel_coords[1], ziel_coords[0])

# ---------------------------------------------------
# 2. Dijkstra-Algorithmus (eigene Implementierung!)
# ---------------------------------------------------
def dijkstra(graph, start, ziel):
    # Initialize distances and previous pointers
    dist = {node: float('inf') for node in graph.nodes}
    dist[start] = 0
    prev = {node: None for node in graph.nodes}
    pq = [(0, start)]

    def edge_length(u, v):
        # Handle MultiDiGraph edges: choose minimal available 'length' or fallback to haversine
        data = graph.get_edge_data(u, v)
        if not data:
            # fallback: compute via node coordinates
            y1 = graph.nodes[u].get('y')
            x1 = graph.nodes[u].get('x')
            y2 = graph.nodes[v].get('y')
            x2 = graph.nodes[v].get('x')
            if None in (x1, y1, x2, y2):
                return 1.0
            return haversine_meters(y1, x1, y2, x2)
        # MultiEdge: data is a dict keyed by edge keys
        if isinstance(data, dict):
            lengths = []
            for key, attr in data.items():
                if isinstance(attr, dict) and 'length' in attr:
                    lengths.append(attr['length'])
            if lengths:
                return min(lengths)
            # no 'length' found, fallback to node coords
            return edge_length_fallback(u, v)
        return 1.0

    def edge_length_fallback(u, v):
        y1 = graph.nodes[u].get('y')
        x1 = graph.nodes[u].get('x')
        y2 = graph.nodes[v].get('y')
        x2 = graph.nodes[v].get('x')
        if None in (x1, y1, x2, y2):
            return 1.0
        return haversine_meters(y1, x1, y2, x2)

    visited = set()
    while pq:
        aktuelle_dist, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if u == ziel:
            break

        for v in graph.neighbors(u):
            laenge = edge_length(u, v)
            alt = dist[u] + laenge
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(pq, (alt, v))

    # reconstruct route
    route = []
    k = ziel
    if prev[k] is None and k != start:
        # no path found
        return []
    while k is not None:
        route.append(k)
        k = prev[k]
    return list(reversed(route))

# ---------------------------------------------------
# Hilfsfunktionen: Geschwindigkeits-/Zeitabschätzung
# ---------------------------------------------------
def _parse_maxspeed_to_kmh(val):
    if val is None:
        return None
    # handle lists/tuples
    if isinstance(val, (list, tuple)):
        # get all numeric candidates, take max
        numbers = []
        for v in val:
            kmh = _parse_maxspeed_to_kmh(v)
            if kmh:
                numbers.append(kmh)
        return max(numbers) if numbers else None
    # handle strings like '50', '50 mph', '30;50', '30-50'
    if isinstance(val, str):
        s = val.lower()
        # detect mph
        is_mph = 'mph' in s
        nums = re.findall(r"\d+", s)
        if not nums:
            return None
        kmh = float(max(nums))
        if is_mph:
            kmh *= 1.60934
        return kmh
    # numeric
    if isinstance(val, (int, float)):
        return float(val)
    return None

def _default_speed_for_highway(h):
    if isinstance(h, (list, tuple)) and h:
        h = h[0]
    h = (str(h).lower() if h is not None else 'unclassified')
    defaults = {
        'motorway': 120.0,
        'trunk': 100.0,
        'primary': 70.0,
        'secondary': 60.0,
        'tertiary': 50.0,
        'unclassified': 40.0,
        'residential': 30.0,
        'service': 20.0,
        'living_street': 10.0,
        'track': 20.0,
        'path': 5.0
    }
    return defaults.get(h, 40.0)

def _edge_speed_kmh(attr, fallback_highway=None):
    # Try explicit maxspeed
    ms = attr.get('maxspeed') if isinstance(attr, dict) else None
    kmh = _parse_maxspeed_to_kmh(ms)
    if kmh is not None and kmh > 0:
        return max(5.0, min(kmh, 130.0))
    # Use highway default
    hw = attr.get('highway') if isinstance(attr, dict) else fallback_highway
    return _default_speed_for_highway(hw)

def estimate_route_time_and_distance(graph, node_list):
    total_len_m = 0.0
    total_sec = 0.0
    for u, v in zip(node_list[:-1], node_list[1:]):
        data = graph.get_edge_data(u, v)
        best_attr = None
        best_len = None
        if data and isinstance(data, dict):
            for k, attr in data.items():
                if not isinstance(attr, dict):
                    continue
                l = attr.get('length')
                if l is None:
                    y1 = graph.nodes[u].get('y'); x1 = graph.nodes[u].get('x')
                    y2 = graph.nodes[v].get('y'); x2 = graph.nodes[v].get('x')
                    if None not in (x1, y1, x2, y2):
                        l = haversine_meters(y1, x1, y2, x2)
                if l is None:
                    continue
                if best_len is None or l < best_len:
                    best_len = l
                    best_attr = attr
        # Fallback if no edge data
        if best_attr is None:
            y1 = graph.nodes[u].get('y'); x1 = graph.nodes[u].get('x')
            y2 = graph.nodes[v].get('y'); x2 = graph.nodes[v].get('x')
            l = haversine_meters(y1, x1, y2, x2)
            speed_kmh = 40.0
        else:
            l = best_len if best_len is not None else 1.0
            speed_kmh = _edge_speed_kmh(best_attr)
        speed_mps = max(1.0, speed_kmh * 1000.0 / 3600.0)
        sec = l / speed_mps
        total_len_m += l
        total_sec += sec
    return total_len_m, total_sec

def _fmt_time_seconds(sec):
    if sec < 30:
        return "< 1 min"
    m = int(round(sec / 60.0))
    return f"{m} min"

def _fmt_dist_m(m):
    km = m / 1000.0
    return f"{km:.1f} km"

# ---------------------------------------------------
# 3. Erzeuge Kanten-GeoDataFrame, berechne Unfall-Risiko und Gewichte
# ---------------------------------------------------
import pandas as pd
import geopandas as gpd
from shapely.geometry import LineString

# Lade Unfalldaten (2024) – nur Siegen/Eiserfeld
acc_file = r"C:\Users\anton\Documents\GitHub\Facharbeit-NAvigation\unfaelle_siegenwittgenstein.csv\Unfallorte2024_LinRef.csv.csv"
try:
    df_acc = pd.read_csv(acc_file, sep=";", decimal=",", low_memory=False)
except Exception as e:
    raise RuntimeError(f"Unfalldatei {acc_file} konnte nicht geladen werden: {e}")

df_acc["XGCSWGS84"] = pd.to_numeric(df_acc["XGCSWGS84"], errors="coerce")
df_acc["YGCSWGS84"] = pd.to_numeric(df_acc["YGCSWGS84"], errors="coerce")
df_acc = df_acc.dropna(subset=["XGCSWGS84", "YGCSWGS84"])

# Filter: nur Unfälle im Bereich Siegen/Eiserfeld (Bounding Box)
min_lon, max_lon = 7.98, 8.08
min_lat, max_lat = 50.82, 50.90
df_acc = df_acc[(df_acc["XGCSWGS84"] >= min_lon) & (df_acc["XGCSWGS84"] <= max_lon) &
                (df_acc["YGCSWGS84"] >= min_lat) & (df_acc["YGCSWGS84"] <= max_lat)]

gdf_acc = gpd.GeoDataFrame(df_acc, geometry=gpd.points_from_xy(df_acc["XGCSWGS84"], df_acc["YGCSWGS84"]), crs="EPSG:4326").to_crs(3857)

# Baue Kanten-GeoDataFrame
rows = []
for u, v, key, data in G.edges(keys=True, data=True):
    geom = None
    if isinstance(data, dict) and "geometry" in data and data["geometry"] is not None:
        geom = data["geometry"]
    else:
        # fallback LineString between node coordinates (lon,lat)
        y1 = G.nodes[u].get("y")
        x1 = G.nodes[u].get("x")
        y2 = G.nodes[v].get("y")
        x2 = G.nodes[v].get("x")
        if None in (x1, y1, x2, y2):
            continue
        geom = LineString([(x1, y1), (x2, y2)])

    length_m = data.get("length") if isinstance(data, dict) and data.get("length") is not None else haversine_meters(G.nodes[u].get("y"), G.nodes[u].get("x"), G.nodes[v].get("y"), G.nodes[v].get("x"))
    highway = None
    if isinstance(data, dict):
        highway = data.get("highway")
    rows.append({"u": u, "v": v, "key": key, "length": length_m, "geometry": geom, "highway": highway})

edges_gdf = gpd.GeoDataFrame(rows, crs="EPSG:4326").to_crs(3857)

# Buffer Kanten und zähle Unfälle in 20m Umgebung
buffers = edges_gdf.copy()
buffers["geometry"] = buffers.geometry.buffer(20)
joined = gpd.sjoin(gdf_acc, buffers[["geometry"]], how="left", predicate="within")
counts = joined.groupby("index_right").size()
edges_gdf["accidents"] = counts.reindex(edges_gdf.index).fillna(0).astype(int)

# Risiko definieren (Unfälle pro km)
edges_gdf["risk"] = edges_gdf.apply(lambda r: r["accidents"] / (max(r["length"], 1) / 1000.0), axis=1)

# Straßenklassen-Strafe basierend auf OSM 'highway' Tag
def highway_penalty_tag(h):
    if h is None:
        return 1.2
    # if list, pick first
    if isinstance(h, (list, tuple)):
        h = h[0]
    mapping = {
        'motorway': 0.6,
        'trunk': 0.7,
        'primary': 0.8,
        'secondary': 0.9,
        'tertiary': 1.0,
        'unclassified': 1.1,
        'residential': 1.5,
        'service': 1.6,
        'living_street': 1.8,
        'track': 2.0,
        'path': 2.5,
        'cycleway': 2.0,
        'footway': 2.5
    }
    return mapping.get(str(h), 1.2)

edges_gdf['road_penalty'] = edges_gdf['highway'].apply(highway_penalty_tag)

# Normalisieren
max_len = edges_gdf["length"].max() or 1.0
max_risk = edges_gdf["risk"].max()
if not max_risk or max_risk <= 0:
    max_risk = 1.0

edges_gdf["len_norm"] = edges_gdf["length"] / max_len
edges_gdf["risk_norm"] = edges_gdf["risk"] / max_risk

# Gewichte – leichte ML-basierte Anpassung zur Bewertung von Straßen (bevorzuge Hauptstraßen)
mix_param = 0.5
route_pref_strength = 1.0  # reset manual strength; AI model will handle preference

# Zielwert (wie "bevorzugt" ist die Straße), basierend auf OSM 'highway' tag
def hw_target(h):
    if h is None:
        return np.nan
    if isinstance(h, (list, tuple)):
        h = h[0]
    h = str(h).lower()
    mapping = {
        'motorway': 1.0,
        'trunk': 0.95,
        'primary': 0.9,
        'secondary': 0.8,
        'tertiary': 0.6,
        'unclassified': 0.4,
        'residential': 0.2,
        'service': 0.15,
        'living_street': 0.1,
        'track': 0.05,
        'path': 0.0
    }
    return mapping.get(h, 0.3)

edges_gdf['hw_target'] = edges_gdf['highway'].apply(hw_target)

# Features: len_norm, risk_norm, accidents, plus highway class encoded
edges_gdf['hw_code'] = edges_gdf['highway'].apply(lambda h: str(h[0]) if isinstance(h, (list, tuple)) and h else (str(h) if h is not None else 'none'))
hw_dummies = pd.get_dummies(edges_gdf['hw_code'], prefix='hw')
X_df = pd.concat([edges_gdf[['len_norm', 'risk_norm', 'accidents']].fillna(0), hw_dummies], axis=1)
X = X_df.values
y = edges_gdf['hw_target'].fillna(-1).values

# Train Ridge with CV if enough samples
mask = y >= 0
if mask.sum() >= 20:
    scaler = StandardScaler()
    Xs = scaler.fit_transform(X[mask])
    alphas = [0.1, 1.0, 10.0]
    model = RidgeCV(alphas=alphas)
    model.fit(Xs, y[mask])
    # predict for all
    Xall = scaler.transform(X)
    pred = model.predict(Xall)
    # normalize predictions to 0..1
    pmin = np.nanmin(pred)
    pmax = np.nanmax(pred)
    if pmax - pmin < 1e-6:
        pref = np.clip(pred, 0.0, 1.0)
    else:
        pref = (pred - pmin) / (pmax - pmin)
else:
    pref = 1.0 - edges_gdf['risk_norm'].fillna(0).values

# Convert preference to highway_weight factor scaled by route_pref_strength
pref = np.clip(pref, 0.0, 1.0)

# --- AI route-decision model: predict per-edge probability that it is a 'main/preferred' road ---
ai_strength = 2.0  # how strongly the AI penalizes non-preferred edges
try:
    # prepare features (reuse X_df created above)
    scaler_ai = StandardScaler().fit(X)
    Xs_ai = scaler_ai.transform(X)
    # binary target derived from hw_target (main roads ~1.0)
    y_bin = (edges_gdf['hw_target'].fillna(0) >= 0.7).astype(int).values
    # require at least some examples of each class
    if (y_bin == 1).sum() >= 10 and (y_bin == 0).sum() >= 10:
        clf = RandomForestClassifier(n_estimators=100, random_state=42)
        clf.fit(Xs_ai[mask], y_bin[mask])
        p_main = clf.predict_proba(Xs_ai)[:, 1]
    else:
        # fallback: use continuous pref (from Ridge) as proxy
        p_main = pref
except Exception:
    p_main = pref

# convert to preference probability (p_main in 0..1)
p_main = np.clip(p_main, 0.0, 1.0)
edges_gdf['ai_pref'] = p_main

# highway weight from previous Ridge prediction (kept for class-based preference)
edges_gdf['highway_weight'] = 1.0 + (1.0 - pref) * (1.0 * route_pref_strength)

# Use per-route AI strength so 'safe' can prioritize main roads more
ai_strengths = {
    'fast': 1.0,
    'safe': 4.0,
    'mix': 2.0
}
edges_gdf['ai_weight_fast'] = 1.0 + (1.0 - p_main) * ai_strengths['fast']
edges_gdf['ai_weight_safe'] = 1.0 + (1.0 - p_main) * ai_strengths['safe']
edges_gdf['ai_weight_mix'] = 1.0 + (1.0 - p_main) * ai_strengths['mix']

# final weights: include road_penalty and per-route AI weight
edges_gdf["weight_fast"] = edges_gdf["length"] * edges_gdf["highway_weight"] * edges_gdf["road_penalty"] * edges_gdf["ai_weight_fast"]
edges_gdf["weight_safe"] = edges_gdf["risk_norm"] * edges_gdf["highway_weight"] * edges_gdf["road_penalty"] * edges_gdf["ai_weight_safe"]
edges_gdf["weight_mix"] = (edges_gdf["len_norm"] * (1 - mix_param) + edges_gdf["risk_norm"] * mix_param) * edges_gdf["highway_weight"] * edges_gdf["road_penalty"] * edges_gdf["ai_weight_mix"]

# Baue gerichteten Graphen mit minimalen Gewichten pro (u,v)
H = nx.DiGraph()
for (u, v), group in edges_gdf.groupby(["u", "v"]):
    w_fast = group["weight_fast"].min()
    w_safe = group["weight_safe"].min()
    w_mix = group["weight_mix"].min()
    H.add_edge(u, v, weight_fast=w_fast, weight_safe=w_safe, weight_mix=w_mix)

# ---------------------------------------------------
# 4. Drei Routen berechnen: schnell, sicher, gemischt
# ---------------------------------------------------
paths = {}
try:
    paths["fast"] = nx.shortest_path(H, source=start, target=ziel, weight="weight_fast")
except Exception:
    paths["fast"] = []
try:
    paths["safe"] = nx.shortest_path(H, source=start, target=ziel, weight="weight_safe")
except Exception:
    paths["safe"] = []
try:
    paths["mix"] = nx.shortest_path(H, source=start, target=ziel, weight="weight_mix")
except Exception:
    paths["mix"] = []

print("Routen gefunden:", {k: (len(v) if v else 0) for k, v in paths.items()})

# Geschätzte Zeit und Distanz für jede Route berechnen
label_map = {"fast": "Schnell", "safe": "Sicher", "mix": "Mischroute"}
route_stats = {}
for kind, node_list in paths.items():
    if not node_list:
        continue
    dist_m, secs = estimate_route_time_and_distance(G, node_list)
    route_stats[kind] = {"meters": dist_m, "seconds": secs}

if route_stats:
    summary = {label_map.get(k, k): f"{_fmt_time_seconds(v['seconds'])}, {_fmt_dist_m(v['meters'])}" for k, v in route_stats.items()}
    print("Zeit/Distanz (ca.):", summary)

# ---------------------------------------------------
# 5. Karte zeichnen (keine Zwischenmarker, Routen entlang Straßengeometrie)
#    + Unfall-Heatmap (für Performance ggf. sampling)
# ---------------------------------------------------
color_map = {"fast": "#1f78b4", "safe": "#33a02c", "mix": "#e31a1c"}
m = folium.Map(location=start_coords, zoom_start=14)
folium.Marker(start_coords, tooltip="Start: Gesamtschule Eiserfeld", icon=folium.Icon(color="green")).add_to(m)
folium.Marker(ziel_coords, tooltip="Ziel: Siegen", icon=folium.Icon(color="red")).add_to(m)

# Unfall-Heatmap (sample falls sehr viele Punkte)
gdf_acc_plot = gdf_acc.to_crs(4326)
n_points = len(gdf_acc_plot)
max_points = 5000
if n_points > max_points:
    sample = gdf_acc_plot.sample(max_points)
else:
    sample = gdf_acc_plot
heat_data = [[pt.y, pt.x] for pt in sample.geometry]
HeatMap(heat_data, radius=7, blur=12, name='Unfall-Heatmap').add_to(m)

def route_coords_from_nodes(node_list):
    coords = []
    for u, v in zip(node_list[:-1], node_list[1:]):
        data = G.get_edge_data(u, v)
        seg_coords = []
        if data:
            if isinstance(data, dict):
                found = False
                for key, attr in data.items():
                    geom = attr.get("geometry") if isinstance(attr, dict) else None
                    if geom is not None:
                        seg_coords = [(lat, lon) for lon, lat in geom.coords]
                        found = True
                        break
                if not found:
                    seg_coords = [(G.nodes[u]["y"], G.nodes[u]["x"]), (G.nodes[v]["y"], G.nodes[v]["x"])]
        else:
            seg_coords = [(G.nodes[u]["y"], G.nodes[u]["x"]), (G.nodes[v]["y"], G.nodes[v]["x"])]

        if not coords:
            coords.extend(seg_coords)
        else:
            coords.extend(seg_coords[1:])
    return coords

for kind, node_list in paths.items():
    if not node_list:
        continue
    coords = route_coords_from_nodes(node_list)
    label = label_map.get(kind, kind)
    stats = route_stats.get(kind)
    tip = label if not stats else f"{label} – ca. {_fmt_time_seconds(stats['seconds'])}, {_fmt_dist_m(stats['meters'])}"
    folium.PolyLine(coords, weight=7, color=color_map.get(kind, "blue"), opacity=0.85, tooltip=tip).add_to(m)

# Layer control
folium.LayerControl().add_to(m)

# Unfälle als Punkte auf der Karte markieren – nur entlang der Route
from shapely.geometry import LineString
from shapely.ops import unary_union
route_lines = []
for kind, node_list in paths.items():
    if not node_list:
        continue
    coords = route_coords_from_nodes(node_list)
    route_lines.append(LineString([(lon, lat) for lat, lon in coords]))
route_union = unary_union(route_lines)
# inner buffer ~30m, outer buffer ~100m (in degrees approx)
inner_buffer = route_union.buffer(0.0003)
outer_buffer = route_union.buffer(0.0009)
gdf_acc_plot = gdf_acc.to_crs(4326)

def val_to_yesno(v):
    if pd.isna(v):
        return 'Unbekannt'
    if isinstance(v, str):
        vv = v.strip().lower()
        if vv in ('1', 'j', 'ja', 'yes', 'true', 'wahr'):
            return 'Ja'
        if vv in ('0', 'n', 'nein', 'no', 'false'):
            return 'Nein'
        return v
    if isinstance(v, (int, float)):
        return 'Ja' if v == 1 else 'Nein'
    return str(v)

vehicle_cols = ["IstRad", "IstPKW", "IstFuss", "IstKrad", "IstGkfz", "IstSonstige"]

# accidents exactly on/very near route
on_route = gdf_acc_plot[gdf_acc_plot.geometry.within(inner_buffer)]
# accidents near route but not inside inner buffer
near_route = gdf_acc_plot[gdf_acc_plot.geometry.within(outer_buffer) & (~gdf_acc_plot.geometry.within(inner_buffer))]

def build_popup(acc):
    lines = []
    if 'UNFALLART' in acc:
        lines.append(f"Art: {acc['UNFALLART']}")
    if 'UNFALLTYP' in acc:
        lines.append(f"Typ: {acc['UNFALLTYP']}")
    if 'STRASSE' in acc:
        lines.append(f"Straße: {acc['STRASSE']}")
    # only show vehicle types with 'Ja'
    involved = []
    for vc in vehicle_cols:
        if vc in acc.index:
            val = val_to_yesno(acc[vc])
            if val == 'Ja':
                # map column names to nicer German labels
                label = vc.replace('Ist', '')
                involved.append(label)
    if involved:
        lines.append("Beteiligte: " + ", ".join(involved))
    # severity / person-related columns
    severity_cols = [c for c in acc.index if ('PERSON' in c.upper() or 'SCHADEN' in c.upper() or 'VERLET' in c.upper())]
    for sc in severity_cols:
        lines.append(f"{sc}: {acc[sc]}")
    for col in ["LICHTVERH", "WETTER", "MONAT", "WOCHENTAG"]:
        if col in acc.index:
            lines.append(f"{col}: {acc[col]}")
    return f"Unfallpunkt:<br>Lat: {acc['YGCSWGS84']}<br>Lon: {acc['XGCSWGS84']}<br>" + "<br>".join(lines)

for _, acc in on_route.iterrows():
    popup_text = build_popup(acc)
    folium.CircleMarker(
        location=[acc.geometry.y, acc.geometry.x],
        radius=5,
        color="black",
        fill=True,
        fill_color="orange",
        fill_opacity=0.95,
        popup=folium.Popup(popup_text, max_width=350)
    ).add_to(m)

for _, acc in near_route.iterrows():
    popup_text = build_popup(acc)
    folium.CircleMarker(
        location=[acc.geometry.y, acc.geometry.x],
        radius=4,
        color="black",
        fill=True,
        fill_color="blue",
        fill_opacity=0.6,
        popup=folium.Popup("In Nähe der Route:<br>" + popup_text, max_width=350)
    ).add_to(m)

# Karte speichern und öffnen
out_path = os.path.abspath("route_eiserfeld_siegen.html")
m.save(out_path)
print(f"Karte gespeichert als {out_path}")
try:
    import webbrowser
    webbrowser.open(f"file://{out_path}")
    print("Versuche, die Karte im Standardbrowser zu öffnen...")
except Exception as e:
    print("Konnte den Browser nicht automatisch öffnen:", e)
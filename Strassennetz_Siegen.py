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
# 2. Dijkstra-Algorithmus 
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

# Lade Unfalldaten (mehrere Jahre möglich)
# Du kannst hier ein Dateipattern (glob) oder einzelne Dateien angeben.
import glob
accident_params = {
    # pattern relative/absolute: suche z.B. "data/Unfallorte*_LinRef.csv" oder ein Ordner
    'files_pattern': r"C:\\Users\\anton\\Documents\\GitHub\\Facharbeit-NAvigation\\unfaelle_siegenwittgenstein.csv\\Unfallorte*_LinRef.csv*",
    # gamma: 1.0 = gleiche Gewichtung, <1 ältere Jahre schwächt ab (z.B. 0.8)
    'gamma': 0.85,
    # optional: explizite Liste von Jahren oder None um alle in den Dateien gefundenen Jahre zu nutzen
    'years': None,
}

# Suche Dateien
files = glob.glob(accident_params['files_pattern'])
if not files:
    # Fallback: versuche die alte hartkodierte Datei (falls vorhanden)
    fallback = r"C:\Users\anton\Documents\GitHub\Facharbeit-NAvigation\unfaelle_siegenwittgenstein.csv\Unfallorte2024_LinRef.csv.csv"
    if os.path.exists(fallback):
        files = [fallback]
    else:
        raise RuntimeError(f"Keine Unfalldateien gefunden mit Pattern {accident_params['files_pattern']} und kein Fallback vorhanden.")

dfs = []
year_re = re.compile(r"(19|20)\d{2}")
for f in files:
    try:
        df = pd.read_csv(f, sep=";", decimal=",", low_memory=False)
    except Exception as e:
        print(f"Warnung: Datei {f} konnte nicht gelesen werden: {e}")
        continue
    # versuche Jahr zu ermitteln: erst aus Spalte 'UJAHR' falls vorhanden, sonst aus Dateiname
    if 'UJAHR' in df.columns:
        df['UJAHR'] = pd.to_numeric(df['UJAHR'], errors='coerce').astype('Int64')
    else:
        m = year_re.search(os.path.basename(f))
        df['UJAHR'] = int(m.group(0)) if m else pd.NA
    dfs.append(df)

if not dfs:
    raise RuntimeError("Keine gültigen Unfalldaten eingelesen.")

df_acc = pd.concat(dfs, ignore_index=True)

# Nur numerische Koordinaten behalten
df_acc["XGCSWGS84"] = pd.to_numeric(df_acc.get("XGCSWGS84", df_acc.get('LINREFX')), errors="coerce")
df_acc["YGCSWGS84"] = pd.to_numeric(df_acc.get("YGCSWGS84", df_acc.get('LINREFY')), errors="coerce")
df_acc = df_acc.dropna(subset=["XGCSWGS84", "YGCSWGS84"])

# Filter: nur Unfälle im Bereich Siegen/Eiserfeld (Bounding Box)
min_lon, max_lon = 7.98, 8.08
min_lat, max_lat = 50.82, 50.90
df_acc = df_acc[(df_acc["XGCSWGS84"] >= min_lon) & (df_acc["XGCSWGS84"] <= max_lon) &
                (df_acc["YGCSWGS84"] >= min_lat) & (df_acc["YGCSWGS84"] <= max_lat)]

# Jahresgewichtung: berechne year_weight per Unfallpunkt
current_year = pd.Timestamp.now().year
gamma = float(accident_params.get('gamma', 1.0) or 1.0)
def _year_weight(y):
    try:
        if pd.isna(y):
            return 1.0
        y = int(y)
        return gamma ** (current_year - y)
    except Exception:
        return 1.0

if 'UJAHR' in df_acc.columns:
    df_acc['year_weight'] = df_acc['UJAHR'].apply(_year_weight)
else:
    df_acc['year_weight'] = 1.0

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
# stelle sicher, dass year_weight existiert
if 'year_weight' not in joined.columns:
    joined['year_weight'] = 1.0

# ungewichtete Anzahl (A_count) und gewichtete Summe (A_weighted)
counts_unweighted = joined.groupby("index_right").size()
counts_weighted = joined.groupby("index_right")['year_weight'].sum()
edges_gdf['A_count'] = counts_unweighted.reindex(edges_gdf.index).fillna(0).astype(int)
edges_gdf['A_weighted'] = counts_weighted.reindex(edges_gdf.index).fillna(0.0)

# Risiko definieren (gewichtete Unfälle pro km)
edges_gdf["risk"] = edges_gdf.apply(lambda r: r["A_weighted"] / (max(r["length"], 1) / 1000.0), axis=1)

# Straßenklassen-Strafe basierend auf OSM 'highway' Tag
def highway_penalty_tag(h):
    if h is None:
        return 1.2
    # if list, pick first
    if isinstance(h, (list, tuple)):
        h = h[0]
    mapping = {
        # Reduced range: values closer to 1 reduce the multiplicative impact
        'motorway': 0.95,
        'trunk': 0.97,
        'primary': 1.00,
        'secondary': 1.03,
        'tertiary': 1.06,
        'unclassified': 1.08,
        'residential': 1.12,
        'service': 1.14,
        'living_street': 1.18,
        'track': 1.20,
        'path': 1.22,
        'cycleway': 1.10,
        'footway': 1.20
    }
    return mapping.get(str(h), 1.2)

edges_gdf['road_penalty'] = edges_gdf['highway'].apply(highway_penalty_tag)

# Normalisieren und berechne Fahrzeit (T) sowie Risiko (R)
mix_param = 0.5

# Länge in km
edges_gdf['length_km'] = edges_gdf['length'] / 1000.0

# Schätze Geschwindigkeit pro Kante über Highway-Typ (Fallback auf Default)
edges_gdf['speed_kmh'] = edges_gdf['highway'].apply(lambda h: _default_speed_for_highway(h))

# Fahrzeit T(e) in Sekunden (vermeidet Division durch 0)
eps = 1e-6
edges_gdf['T_sec'] = edges_gdf.apply(lambda r: (r['length_km'] / max(eps, r['speed_kmh'])) * 3600.0, axis=1)

# Risiko R(e) = beta(e) * (A_weighted + alpha) / length_km
# alpha wird konstant auf 1 gesetzt (Glättung), beta nutzen wir aus 'road_penalty'
alpha = 1
edges_gdf['R'] = edges_gdf.apply(lambda r: (r['road_penalty'] * (r['A_weighted'] + alpha) / max(eps, r['length_km'])) if r['length_km'] > 0 else 0.0, axis=1)

# Normiere auf 0..1
T_max = edges_gdf['T_sec'].max() or 1.0
R_max = edges_gdf['R'].max() or 1.0
edges_gdf['T_norm'] = edges_gdf['T_sec'] / T_max
edges_gdf['R_norm'] = edges_gdf['R'] / R_max

# --- User-configurable route weighting parameters ---
# Für jede Route kannst du hier direkt einstellen:
#  - t_weight: Anteil / Skalar für T (normiert)
#  - r_weight: Anteil / Skalar für R (normiert)
#  - t_scale, r_scale: zusätzliche Skalierung vor Kombination
#  - use_road_penalty: ob road_penalty (zusätzlich zur bereits in R verwendeten beta) multipliziert werden soll
#  - road_penalty_mul: zusätzlicher Multiplikator
# Beispiel: setze r_weight hoch für 'safe', t_weight hoch für 'fast'
route_params = {
    'fast': {'t_weight': 1.0, 'r_weight': 0.0, 't_scale': 1.0, 'r_scale': 1.0, 'use_road_penalty': False, 'road_penalty_mul': 1.0},
    'safe': {'t_weight': 0.0, 'r_weight': 1.0, 't_scale': 1.0, 'r_scale': 1.0, 'use_road_penalty': False, 'road_penalty_mul': 1.0},
    'mix':  {'t_weight': 1.0 - mix_param, 'r_weight': mix_param, 't_scale': 1.0, 'r_scale': 1.0, 'use_road_penalty': False, 'road_penalty_mul': 1.0}
}

# Berechne finale Gewichte aus den konfigurierten Parametern
for kind, p in route_params.items():
    base = p['t_weight'] * p.get('t_scale', 1.0) * edges_gdf['T_norm'] + p['r_weight'] * p.get('r_scale', 1.0) * edges_gdf['R_norm']
    if p.get('use_road_penalty', False):
        base = base * edges_gdf['road_penalty'] * p.get('road_penalty_mul', 1.0)
    edges_gdf[f'weight_{kind}'] = base

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

# --- Diagnose: prüfe, ob mix mit safe/fast übereinstimmt und berechne Gesamt-R für jede Route ---
def route_total_R(node_list):
    if not node_list:
        return 0.0
    total_R = 0.0
    for u, v in zip(node_list[:-1], node_list[1:]):
        subset = edges_gdf[(edges_gdf['u'] == u) & (edges_gdf['v'] == v)]
        if subset.empty:
            continue
        # wähle minimalen R falls parallelkanten
        total_R += subset['R'].min()
    return total_R

for kind, node_list in paths.items():
    if not node_list:
        continue
    t = route_stats.get(kind, {}).get('seconds', None)
    meters = route_stats.get(kind, {}).get('meters', None)
    rsum = route_total_R(node_list)
    print(f"Route {kind}: Knoten={len(node_list)}, Zeit(s)={t}, Distanz(m)={meters}, Sum_R={rsum:.3f}")

print("mix == safe:", paths.get('mix') == paths.get('safe'))
print("mix == fast:", paths.get('mix') == paths.get('fast'))

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
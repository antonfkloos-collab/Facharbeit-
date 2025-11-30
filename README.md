# Facharbeit – Graphen und Straßennetz

Dieses Repo enthält zwei voneinander unabhängige Python-Skripte:

- `Graphen.py`: Interaktives Tkinter-Tool zur Visualisierung und Routenberechnung auf einem selbst erstellten Graphen (optional mit Pillow für glatte Darstellung).
- `Straßennetz.py` (alias `Strassennetz.py`): Routenberechnung auf realen Straßennetzen mit OSMnx, Folium-Karte als HTML-Ausgabe. Nutzt Unfallpunkte aus CSV.

## Voraussetzungen

- Python 3.10–3.12
- Abhängigkeiten installieren:

```pwsh
python -m pip install -r requirements.txt
```

Hinweis zu Geo-Stack (Geopandas/Shapely/Rtree): Unter Windows/macOS sollten Wheels automatisch installiert werden. Falls Probleme auftreten, bitte aktuelle Python-Version und `pip` verwenden.

## Graphen-Tool starten

```pwsh
python Graphen.py
```

Optional ohne Pillow (langsameres Rendering): funktioniert trotzdem.

## Straßennetz-Tool starten (CLI)

Daten vorbereiten:
- Lege die Unfalldatei (CSV) in `data/Unfallorte2024_LinRef.csv.csv` ab, oder übergib den Pfad via `--acc-file`.

Beispiel (Standard-Region Siegen, Standard-Start/Ziel):

```pwsh
python Straßennetz.py --acc-file data/Unfallorte2024_LinRef.csv.csv
```

Alternative mit ASCII-Dateiname (falls Sonderzeichen-Probleme):

```pwsh
python Strassennetz.py --acc-file data/Unfallorte2024_LinRef.csv.csv
```

Weitere Optionen:

```pwsh
python Straßennetz.py \
  --place "Siegen, Germany" \
  --start "Gesamtschule Eiserfeld, Siegen, Germany" \
  --ziel  "Siegen ZOB, Siegen, Germany" \
  --acc-file data/Unfallorte2024_LinRef.csv.csv \
  --output route_eiserfeld_siegen.html
```

Nach dem Lauf findest du die Karte als HTML-Datei (z. B. `route_eiserfeld_siegen.html`). Diese kann in jedem Browser geöffnet werden (auch iPad via Dateien-App oder iCloud Drive).

## iPad-Hinweise

- Ansehen/Bearbeiten: Öffne das Repo auf GitHub (Browser oder App) oder nutze GitHub Codespaces.
- Ohne Git: Exportiere das Repo als ZIP, entpacke es in der Dateien-App. Öffne HTML-Ergebnisse direkt im mobilen Browser.
- Editor-Probleme mit `ß`: Nutze `Strassennetz.py` (ASCII-Variante).

## Lizenz

Nur zu Schul-/Projektzwecken. Externe Daten (Unfalldatei) bitte separat bereitstellen.

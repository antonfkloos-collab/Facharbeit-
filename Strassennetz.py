# ASCII-kompatible Wrapper-Datei für Systeme/Apps mit Problemen bei Sonderzeichen im Dateinamen.
from pathlib import Path
code_path = Path(__file__).with_name('Straßennetz.py')
code = code_path.read_text(encoding='utf-8')
exec(compile(code, str(code_path), 'exec'))

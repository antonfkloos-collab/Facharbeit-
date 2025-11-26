import tkinter as tk
from tkinter import messagebox
import math
import heapq
import random
try:
    from PIL import Image, ImageDraw, ImageTk
except Exception:
    Image = None
    ImageDraw = None
    ImageTk = None

class GraphTool:
    def __init__(self, root):
        self.root = root
        self.root.title("Straßennetz – Dijkstra")
        self.root.geometry("1200x800")
        self.root.configure(bg="#F7F8FC")
        
        # Graph-Daten
        self.nodes = {}  # {id: (x, y)}
        self.edges = {}  # {(node1, node2): weight}
        self.node_counter = 0
        
        # Zustände
        self.mode = "add_node"  # add_node, add_edge, set_start, set_goal
        self.selected_node = None
        self.start_node = None
        self.goal_node = None
        self.path = []
        # Auswahl der Routen-Variante: fast | safe | mix
        self.route_choice = tk.StringVar(value="fast")
        # Unfalldichte (0.0–1.0): steuert, wie viele Unfallpunkte angezeigt werden
        self.acc_density = 0.7
        
        self.setup_ui()
        
    def setup_ui(self):
        # Toolbar (hell, schlicht)
        toolbar = tk.Frame(self.root, bg="#FAFAFF", height=64)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        button_style = {
            "font": ("Segoe UI", 10, "bold"),
            "relief": tk.SOLID,
            "bd": 1,
            "padx": 16,
            "pady": 10,
            "cursor": "hand2",
            "bg": "#EDEBFF",
            "fg": "#16223A",
            "activebackground": "#DCD8FF",
            "activeforeground": "#16223A",
            "highlightthickness": 0
        }
        
        # Buttons in gewünschter Reihenfolge anordnen
        self.btn_add_node = tk.Button(toolbar, text="Knoten hinzufügen", 
                                       command=lambda: self.set_mode("add_node"),
                                       **button_style)
        self.btn_add_node.pack(side=tk.LEFT, padx=5)

        self.btn_add_edge = tk.Button(toolbar, text="Kante hinzufügen", 
                                       command=lambda: self.set_mode("add_edge"),
                                       **button_style)
        self.btn_add_edge.pack(side=tk.LEFT, padx=5)

        self.btn_add_accident = tk.Button(toolbar, text="Unfallpunkt hinzufügen", 
            command=lambda: self.set_mode("add_accident"),
            **button_style)
        self.btn_add_accident.pack(side=tk.LEFT, padx=5)

        self.btn_set_start = tk.Button(toolbar, text="Startpunkt", 
                                        command=lambda: self.set_mode("set_start"),
                                        **button_style)
        self.btn_set_start.pack(side=tk.LEFT, padx=5)

        self.btn_set_goal = tk.Button(toolbar, text="Zielpunkt", 
                                       command=lambda: self.set_mode("set_goal"),
                                       **button_style)
        self.btn_set_goal.pack(side=tk.LEFT, padx=5)

        # Zufallsnetz erzeugen
        self.btn_random = tk.Button(toolbar, text="Zufallsnetz", 
                        command=self.generate_random_graph,
                        **button_style)
        self.btn_random.pack(side=tk.LEFT, padx=5)

        self.btn_clear = tk.Button(toolbar, text="Zurücksetzen", 
                                    command=self.clear_all,
                                    **button_style)
        self.btn_clear.pack(side=tk.LEFT, padx=5)

        # Unfalldichte-Regler
        density_box = tk.Frame(toolbar, bg="#FFFFFF", relief=tk.SOLID, bd=1, highlightthickness=0)
        density_box.pack(side=tk.LEFT, padx=10)
        tk.Label(density_box, text="Unfalldichte", bg="#FFFFFF", fg="#16223A", font=("Segoe UI", 10, "bold")).pack(side=tk.TOP, anchor="w")
        self.density_scale = tk.Scale(density_box, from_=0, to=100, orient=tk.HORIZONTAL, length=160,
                          bg="#FFFFFF", highlightthickness=0, troughcolor="#EDEBFF",
                                      showvalue=False, command=self._on_density_change)
        self.density_scale.set(int(self.acc_density * 100))
        self.density_scale.pack(side=tk.TOP)

        # Auswahl der Routenart – jede Option in eigenem Kasten
        rb_kwargs = {"bg": "#FFFFFF", "activebackground": "#FFFFFF", "selectcolor": "#EDEBFF", "font": ("Segoe UI", 10, "bold")}
        fast_box = tk.Frame(toolbar, bg="#FFFFFF", relief=tk.SOLID, bd=1, highlightthickness=0)
        fast_box.pack(side=tk.LEFT, padx=6)
        tk.Radiobutton(fast_box, text="Schnell", variable=self.route_choice, value="fast", **rb_kwargs).pack(side=tk.LEFT, padx=8, pady=6)

        safe_box = tk.Frame(toolbar, bg="#FFFFFF", relief=tk.SOLID, bd=1, highlightthickness=0)
        safe_box.pack(side=tk.LEFT, padx=6)
        tk.Radiobutton(safe_box, text="Sicher", variable=self.route_choice, value="safe", **rb_kwargs).pack(side=tk.LEFT, padx=8, pady=6)

        mix_box = tk.Frame(toolbar, bg="#FFFFFF", relief=tk.SOLID, bd=1, highlightthickness=0)
        mix_box.pack(side=tk.LEFT, padx=6)
        tk.Radiobutton(mix_box, text="Misch", variable=self.route_choice, value="mix", **rb_kwargs).pack(side=tk.LEFT, padx=8, pady=6)

        # Route berechnen (am Ende)
        self.btn_calculate = tk.Button(toolbar, text="Route berechnen", 
                        command=self.calculate_path,
                        **button_style)
        self.btn_calculate.config(bg="#6B5EFA", fg="#ffffff", activebackground="#4F43D9", activeforeground="#ffffff")
        self.btn_calculate.pack(side=tk.LEFT, padx=5)

        # Status-Label
        self.status_label = tk.Label(toolbar, text="Modus: Knoten hinzufügen", 
                         font=("Segoe UI", 11, "bold"),
                         bg="#ffffff", fg="#1A1A1A", padx=20)
        self.status_label.pack(side=tk.RIGHT)
        
        # Canvas in "Card"-Frame mit feiner Kontur
        canvas_card = tk.Frame(self.root, bg="#FFFFFF", highlightthickness=0)
        canvas_card.pack(fill=tk.BOTH, expand=True, padx=16, pady=12)

        self.canvas = tk.Canvas(canvas_card, bg="#FFFFFF", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.canvas_click)
        # Erste Zeichnung nach Layout
        self.root.after(50, self.draw_graph)

    def _on_density_change(self, val):
        try:
            v = float(val) / 100.0
        except Exception:
            v = 0.5
        self.acc_density = max(0.0, min(1.0, v))
        self.draw_graph()

    def _acc_indices(self, acc_list):
        m = len(acc_list)
        if m == 0:
            return []
        p = min(1.0, (self.acc_density) * (3.0 / (m + 1)))
        show_n = 0 if self.acc_density <= 0 else max(1, int(round(p * m)))
        if show_n <= 0:
            return []
        return [min(m - 1, int(k * m / show_n)) for k in range(show_n)]
        
    def set_mode(self, mode):
        self.mode = mode
        self.selected_node = None
        
        # Button-Farben zurücksetzen
        buttons = {
            "add_node": (self.btn_add_node, "Knoten hinzufügen"),
            "add_edge": (self.btn_add_edge, "Kante hinzufügen"),
            "set_start": (self.btn_set_start, "Startpunkt setzen"),
            "set_goal": (self.btn_set_goal, "Zielpunkt setzen"),
            "add_accident": (self.btn_add_accident, "Unfallpunkt hinzufügen")
        }
        
        for btn_mode, (btn, text) in buttons.items():
            if btn_mode == mode:
                # Aktiven Button dunkel
                btn.config(bg="#1A1A1A", fg="#ffffff", relief=tk.SUNKEN)
                self.status_label.config(text=f"Modus: {text}", fg="#1A1A1A")
            else:
                btn.config(bg="#F1F1FC", fg="#1A1A1A", relief=tk.SOLID)
    
    def add_node(self, x, y):
        self.nodes[self.node_counter] = (x, y)
        self.node_counter += 1
        self.draw_graph()
    
    def add_edge(self, node1, node2):
        weight = self.pick_weight()
        if weight is not None:
            w = int(weight)
            self.edges[(node1, node2)] = {'weight': w, 'accidents': []}
            self.edges[(node2, node1)] = {'weight': w, 'accidents': []}  # Bidirektional
            self.draw_graph()

    def generate_random_graph(self, n_nodes: int = 12, extra_edges: int = 6):
        """Erzeugt ein kleines verbundenes Zufalls-Straßennetz.
        - n_nodes: Anzahl Knoten (klein halten)
        - Jeder Kante bekommt Gewicht 1–10 und 1–10 Unfälle (als Punkte auf der Kante)
        """
        # Canvas-Größe ermitteln
        self.root.update_idletasks()
        w = max(600, self.canvas.winfo_width() or 800)
        h = max(400, self.canvas.winfo_height() or 600)
        margin = 40

        # Reset
        self.nodes = {}
        self.edges = {}
        self.node_counter = 0
        self.selected_node = None
        self.start_node = None
        self.goal_node = None
        self.path = []

        # Knoten gleichmäßig verteilen (Jitter-Grid statt Haufenbildung)
        cols = max(2, int(round(n_nodes ** 0.5)))
        rows = max(2, (n_nodes + cols - 1) // cols)
        cell_w = max(1, (w - 2 * margin) // cols)
        cell_h = max(1, (h - 2 * margin) // rows)
        i = 0
        for r in range(rows):
            for c in range(cols):
                if i >= n_nodes:
                    break
                jitter_x = int(0.2 * cell_w) + random.randint(0, max(1, int(0.6 * cell_w)))
                jitter_y = int(0.2 * cell_h) + random.randint(0, max(1, int(0.6 * cell_h)))
                x = margin + c * cell_w + min(cell_w - 1, jitter_x)
                y = margin + r * cell_h + min(cell_h - 1, jitter_y)
                self.nodes[i] = (x, y)
                i += 1
        self.node_counter = n_nodes

        # Alle Kanten mit Distanz vorbereiten
        all_edges = []  # (dist2, u, v)
        for i in range(n_nodes):
            x1, y1 = self.nodes[i]
            for j in range(i+1, n_nodes):
                x2, y2 = self.nodes[j]
                d2 = (x1-x2)**2 + (y1-y2)**2
                all_edges.append((d2, i, j))
        all_edges.sort(key=lambda t: t[0])

        # DSU/Union-Find für Verbundenheit
        parent = list(range(n_nodes))
        rank = [0]*n_nodes
        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        def union(a,b):
            ra, rb = find(a), find(b)
            if ra == rb:
                return False
            if rank[ra] < rank[rb]:
                parent[ra] = rb
            elif rank[rb] < rank[ra]:
                parent[rb] = ra
            else:
                parent[rb] = ra
                rank[ra] += 1
            return True

        # Baue ein Spannbaum (Kruskal)
        undirected_added = set()
        for d2, u, v in all_edges:
            if union(u, v):
                undirected_added.add((min(u,v), max(u,v)))
                # Kante mit Zufallsgewicht + Unfällen anlegen (beidseitig)
                w_val = random.randint(1, 10)
                self.edges[(u, v)] = {'weight': w_val, 'accidents': []}
                self.edges[(v, u)] = {'weight': w_val, 'accidents': []}
                # Unfälle generieren (1–10 Punkte auf der Kante)
                acc_n = random.randint(1, 10)
                x1, y1 = self.nodes[u]; x2, y2 = self.nodes[v]
                for _ in range(acc_n):
                    t = random.random()
                    px = x1 + t*(x2 - x1)
                    py = y1 + t*(y2 - y1)
                    self.edges[(u, v)]['accidents'].append((px, py))
                    self.edges[(v, u)]['accidents'].append((px, py))
            if len(undirected_added) == n_nodes - 1:
                break

        # Zusätzliche Kanten für Dichte (ohne Kreuzungen)
        added_extra = 0
        for d2, u, v in all_edges:
            if added_extra >= extra_edges:
                break
            key = (min(u,v), max(u,v))
            if key in undirected_added:
                continue
            # Neue Kante nur, wenn sie bestehende nicht schneidet
            if self._edge_would_cross(u, v, undirected_added):
                continue
            undirected_added.add(key)
            w_val = random.randint(1, 10)
            self.edges[(u, v)] = {'weight': w_val, 'accidents': []}
            self.edges[(v, u)] = {'weight': w_val, 'accidents': []}
            acc_n = random.randint(1, 10)
            x1, y1 = self.nodes[u]; x2, y2 = self.nodes[v]
            for _ in range(acc_n):
                t = random.random()
                px = x1 + t*(x2 - x1)
                py = y1 + t*(y2 - y1)
                self.edges[(u, v)]['accidents'].append((px, py))
                self.edges[(v, u)]['accidents'].append((px, py))
            added_extra += 1

        self.status_label.config(text="Modus: Zufallsnetz erzeugt – Start/Ziel wählen")
        self.draw_graph()

    def add_accident_to_edge(self, node1, node2, x, y):
        # Unfallpunkt exakt auf die Linie projizieren
        x1, y1 = self.nodes[node1]
        x2, y2 = self.nodes[node2]
        dx, dy = x2 - x1, y2 - y1
        if dx == dy == 0:
            px, py = x1, y1
        else:
            t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
            px, py = x1 + t * dx, y1 + t * dy
        if (node1, node2) in self.edges:
            self.edges[(node1, node2)]['accidents'].append((px, py))
        if (node2, node1) in self.edges:
            self.edges[(node2, node1)]['accidents'].append((px, py))
        self.draw_graph()

    # Geometrie-Helfer: Kantenkreuzungen verhindern
    def _orient(self, ax, ay, bx, by, cx, cy):
        return (by - ay) * (cx - bx) - (bx - ax) * (cy - by)

    def _on_segment(self, ax, ay, bx, by, cx, cy):
        return min(ax, bx) <= cx <= max(ax, bx) and min(ay, by) <= cy <= max(ay, by)

    def _segments_intersect(self, a, b, c, d):
        ax, ay = a; bx, by = b; cx, cy = c; dx, dy = d
        o1 = self._orient(ax, ay, bx, by, cx, cy)
        o2 = self._orient(ax, ay, bx, by, dx, dy)
        o3 = self._orient(cx, cy, dx, dy, ax, ay)
        o4 = self._orient(cx, cy, dx, dy, bx, by)
        if (o1 > 0 and o2 < 0 or o1 < 0 and o2 > 0) and (o3 > 0 and o4 < 0 or o3 < 0 and o4 > 0):
            return True
        if o1 == 0 and self._on_segment(ax, ay, bx, by, cx, cy):
            return True
        if o2 == 0 and self._on_segment(ax, ay, bx, by, dx, dy):
            return True
        if o3 == 0 and self._on_segment(cx, cy, dx, dy, ax, ay):
            return True
        if o4 == 0 and self._on_segment(cx, cy, dx, dy, bx, by):
            return True
        return False

    def _edge_would_cross(self, u, v, undirected_edges):
        x1, y1 = self.nodes[u]; x2, y2 = self.nodes[v]
        A = (x1, y1); B = (x2, y2)
        for (a, b) in undirected_edges:
            if u in (a, b) or v in (a, b):
                continue
            xa, ya = self.nodes[a]; xb, yb = self.nodes[b]
            C = (xa, ya); D = (xb, yb)
            if self._segments_intersect(A, B, C, D):
                return True
        return False

    def pick_weight(self):
        """Öffnet einen minimalistischen Dialog mit 1–10 als Klickleiste."""
        top = tk.Toplevel(self.root)
        top.title("Kantengewicht wählen")
        top.configure(bg="#ffffff")
        top.transient(self.root)
        top.grab_set()

        # Layout
        container = tk.Frame(top, bg="#ffffff")
        container.pack(padx=16, pady=16)

        title = tk.Label(container, text="Gewicht auswählen", bg="#ffffff", fg="#111827", font=("Segoe UI", 11, "bold"))
        title.pack(anchor="w", pady=(0, 8))

        bar = tk.Frame(container, bg="#ffffff", highlightthickness=1, highlightbackground="#e5e7eb")
        bar.pack(fill=tk.X)

        selected = {"value": None}

        def choose(v):
            selected["value"] = v
            top.destroy()

        for i in range(1, 11):
            b = tk.Button(bar, text=str(i), width=3, pady=8, bg="#ffffff", fg="#111827", relief=tk.SOLID, bd=1,
                          activebackground="#111827", activeforeground="#ffffff",
                          command=lambda v=i: choose(v))
            b.grid(row=0, column=i-1, padx=(0 if i == 1 else 1), pady=1, sticky="nsew")
            bar.grid_columnconfigure(i-1, weight=1)

        # Abbrechen
        actions = tk.Frame(container, bg="#ffffff")
        actions.pack(fill=tk.X, pady=(12, 0))
        cancel = tk.Button(actions, text="Abbrechen", bg="#ffffff", fg="#111827", relief=tk.SOLID, bd=1, padx=12, pady=6,
                           command=top.destroy)
        cancel.pack(side=tk.RIGHT)

        # Zentrieren
        top.update_idletasks()
        w, h = top.winfo_width(), top.winfo_height()
        rx = self.root.winfo_rootx()
        ry = self.root.winfo_rooty()
        rw = self.root.winfo_width()
        rh = self.root.winfo_height()
        x = rx + (rw - w) // 2
        y = ry + (rh - h) // 2
        top.geometry(f"+{max(0, x)}+{max(0, y)}")

        top.bind("<Escape>", lambda _: top.destroy())
        top.wait_window()
        return selected["value"]
    
    def get_node_at(self, x, y, radius=18):
        for node_id, (nx, ny) in self.nodes.items():
            dist = math.sqrt((x - nx)**2 + (y - ny)**2)
            if dist <= radius:
                return node_id
        return None
    
    def highlight_node(self, node_id):
        x, y = self.nodes[node_id]
        # Highlight-Ring in Startfarbe (statt Zielviolett)
        self.canvas.create_oval(x-25, y-25, x+25, y+25,
                                 outline="#7D3692", width=3, tags="highlight")
    
    def draw_graph(self):
        # Double-Buffered Rendering mit Pillow für glatte Kanten/Kreise
        self.canvas.delete("all")

        w = max(1, self.canvas.winfo_width())
        h = max(1, self.canvas.winfo_height())
        SCALE = 2

        if Image is None:
            # Fallback ohne Pillow
            self._draw_graph_fallback()
            return

        img = Image.new("RGB", (w * SCALE, h * SCALE), "#FFFFFF")
        dr = ImageDraw.Draw(img)

        col_edge = "#D9D6EF"   # zartes Violettgrau
        col_path = "#5CC5A7"   # freundliches Grünblau
        col_outline = "#273043"
        r_node = 15

        # Pfad
        if self.path:
            for i in range(len(self.path) - 1):
                x1, y1 = self.nodes[self.path[i]]
                x2, y2 = self.nodes[self.path[i + 1]]
                dr.line([(x1 * SCALE, y1 * SCALE), (x2 * SCALE, y2 * SCALE)], fill=col_path, width=5 * SCALE)

        # Kanten und Label-Container sammeln
        label_boxes = []  # (x,y,text)
        for (node1, node2), data in self.edges.items():
            if node1 < node2:
                x1, y1 = self.nodes[node1]
                x2, y2 = self.nodes[node2]
                dr.line([(x1 * SCALE, y1 * SCALE), (x2 * SCALE, y2 * SCALE)], fill=col_edge, width=2 * SCALE)
                # Unfallpunkte (gedünnt) zeichnen
                acc_list = data['accidents']
                for idx in self._acc_indices(acc_list):
                    ax, ay = acc_list[idx]
                    dr.ellipse([(ax - 6) * SCALE, (ay - 6) * SCALE, (ax + 6) * SCALE, (ay + 6) * SCALE], fill="#FF9FB3", outline=None)
                mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
                # Runde Label-Box im Hintergrund
                box_w, box_h, rad = 36, 22, 10
                x0 = (mid_x - box_w / 2) * SCALE
                y0 = (mid_y - box_h / 2) * SCALE
                x1b = (mid_x + box_w / 2) * SCALE
                y1b = (mid_y + box_h / 2) * SCALE
                dr.rounded_rectangle([x0, y0, x1b, y1b], radius=rad * SCALE, fill="#F5F3FF", outline="#D9D6EF", width=1 * SCALE)
                label_boxes.append((mid_x, mid_y, str(int(data['weight']))) )

        # Knoten
        for node_id, (x, y) in self.nodes.items():
            base = "#EDEBFF"
            if node_id == self.start_node:
                base = "#6B5EFA"
            elif node_id == self.goal_node:
                base = "#86A8FF"

            x0 = (x - r_node) * SCALE
            y0 = (y - r_node) * SCALE
            x1c = (x + r_node) * SCALE
            y1c = (y + r_node) * SCALE
            dr.ellipse([x0, y0, x1c, y1c], fill=base, outline=col_outline, width=1 * SCALE)

        # Downsample für Anti-Aliasing
        img_small = img.resize((w, h), Image.LANCZOS)
        self._imgtk = ImageTk.PhotoImage(img_small)
        self.canvas.create_image(0, 0, anchor="nw", image=self._imgtk)

        # Texte als Overlay (Canvas für gute Schrift)
        for node_id, (x, y) in self.nodes.items():
            txt_color = "#ffffff" if node_id in (self.start_node, self.goal_node) else "#16223A"
            self.canvas.create_text(x, y, text=str(node_id), fill=txt_color, font=("Segoe UI", 10, "bold"))

        for (mx, my, txt) in label_boxes:
            self.canvas.create_text(mx, my, text=txt, fill="#16223A", font=("Segoe UI", 9, "bold"))

    def _draw_graph_fallback(self):
        """Fallback ohne Pillow (keine Glättung)."""
        # Pfad
        if self.path:
            for i in range(len(self.path) - 1):
                x1, y1 = self.nodes[self.path[i]]
                x2, y2 = self.nodes[self.path[i + 1]]
                self.canvas.create_line(x1, y1, x2, y2, fill="#796C61", width=5)

        for (node1, node2), data in self.edges.items():
            if node1 < node2:
                x1, y1 = self.nodes[node1]
                x2, y2 = self.nodes[node2]
                self.canvas.create_line(x1, y1, x2, y2, fill="#D9D6EF", width=2)
                acc_list = data['accidents']
                for idx in self._acc_indices(acc_list):
                    ax, ay = acc_list[idx]
                    self.canvas.create_oval(ax-6, ay-6, ax+6, ay+6, fill="#FF9FB3", outline="")
                mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
                # Ovale (Pill-)Label statt Rechteck
                self.canvas.create_oval(mid_x-20, mid_y-12, mid_x+20, mid_y+12, fill="#F5F3FF", outline="#D9D6EF")
                self.canvas.create_text(mid_x, mid_y, text=str(int(data['weight'])), fill="#16223A", font=("Segoe UI", 9, "bold"))

        for node_id, (x, y) in self.nodes.items():
            base = "#EDEBFF"
            if node_id == self.start_node:
                base = "#6B5EFA"
            elif node_id == self.goal_node:
                base = "#86A8FF"
            self.canvas.create_oval(x-15, y-15, x+15, y+15, fill=base, outline="#273043", width=1)
            txt_color = "#ffffff" if node_id in (self.start_node, self.goal_node) else "#16223A"
            self.canvas.create_text(x, y, text=str(node_id), fill=txt_color, font=("Segoe UI", 10, "bold"))
    
    def calculate_path(self):
        if self.start_node is None or self.goal_node is None:
            messagebox.showwarning("Warnung", "Bitte Start- und Zielpunkt festlegen!")
            return

        def dijkstra(cost_func):
            distances = {node: float('inf') for node in self.nodes}
            distances[self.start_node] = 0
            previous = {node: None for node in self.nodes}
            pq = [(0, self.start_node)]
            visited = set()
            while pq:
                current_dist, current_node = heapq.heappop(pq)
                if current_node in visited:
                    continue
                visited.add(current_node)
                if current_node == self.goal_node:
                    break
                for (n1, n2), data in self.edges.items():
                    neighbor = None
                    if n1 == current_node:
                        neighbor = n2
                    elif n2 == current_node:
                        neighbor = n1
                    if neighbor and neighbor not in visited:
                        cost = cost_func(data)
                        new_dist = current_dist + cost
                        if new_dist < distances[neighbor]:
                            distances[neighbor] = new_dist
                            previous[neighbor] = current_node
                            heapq.heappush(pq, (new_dist, neighbor))
            # Pfad rekonstruieren
            path = []
            if distances[self.goal_node] != float('inf'):
                current = self.goal_node
                while current is not None:
                    path.insert(0, current)
                    current = previous[current]
            return path, distances[self.goal_node]

        # Kostenfunktionen mit Normierung, damit "Misch" nicht faktisch "Sicher" ist
        all_weights = [max(1, d['weight']) for d in self.edges.values()] or [1]
        all_accs = [len(d['accidents']) for d in self.edges.values()] or [0]
        w_max = max(all_weights) if all_weights else 1
        a_max = max(all_accs) if all_accs else 1
        def w_norm(d):
            return (max(1, d['weight'])) / w_max
        def a_norm(d):
            return (len(d['accidents'])) / a_max if a_max > 0 else 0.0

        # Schnell: nahezu nur Länge, minimale Unfall-Tie-Breaker
        def cost_fast(data):
            return 0.9 * w_norm(data) + 0.1 * a_norm(data)
        # Sicher: strikt Unfälle minimieren (exakte Summe), kein Einfluss der Länge
        def cost_safe(data):
            return len(data['accidents'])
        # Misch: echte Balance, nicht zu unfall-lastig
        def cost_mix(data):
            return 0.55 * w_norm(data) + 0.45 * a_norm(data)

        path_fast, dist_fast = dijkstra(cost_fast)
        path_safe, dist_safe = dijkstra(cost_safe)
        path_mix, dist_mix = dijkstra(cost_mix)

        self.path_fast = path_fast
        self.path_safe = path_safe
        self.path_mix = path_mix

        # Nur die ausgewählte Variante anzeigen
        choice = self.route_choice.get()
        if choice == "fast":
            if path_fast:
                messagebox.showinfo("Route – Schnell", f"Schnellste Route:\n{' → '.join(map(str, path_fast))}\nKosten: {dist_fast:.2f}")
                self.path = self.path_fast
            else:
                messagebox.showinfo("Route – Schnell", "Kein Pfad gefunden!")
                self.path = []
        elif choice == "safe":
            if path_safe:
                messagebox.showinfo("Route – Sicher", f"Sicherste Route:\n{' → '.join(map(str, path_safe))}\nUnfälle (Summe pro Kante): {dist_safe:.0f}")
                self.path = self.path_safe
            else:
                messagebox.showinfo("Route – Sicher", "Kein Pfad gefunden!")
                self.path = []
        else:
            if path_mix:
                messagebox.showinfo("Route – Misch", f"Gemischte Route:\n{' → '.join(map(str, path_mix))}\nKosten: {dist_mix:.2f}")
                self.path = self.path_mix
            else:
                messagebox.showinfo("Route – Misch", "Kein Pfad gefunden!")
                self.path = []
        self.draw_graph()
    
    def clear_all(self):
        self.nodes = {}
        self.edges = {}
        self.node_counter = 0
        self.selected_node = None
        self.start_node = None
        self.goal_node = None
        self.path = []
        self.draw_graph()

    def get_edge_at(self, x, y, tolerance=10):
        for (node1, node2), data in self.edges.items():
            x1, y1 = self.nodes[node1]
            x2, y2 = self.nodes[node2]
            dx, dy = x2 - x1, y2 - y1
            if dx == dy == 0:
                continue
            t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
            px, py = x1 + t * dx, y1 + t * dy
            dist = math.hypot(x - px, y - py)
            if dist <= tolerance:
                return (node1, node2)
        return None

    def canvas_click(self, event):
        x, y = event.x, event.y
        if self.mode == "add_accident":
            edge = self.get_edge_at(x, y)
            if edge:
                self.add_accident_to_edge(*edge, x, y)
            return

        if event.num == 3:  # Rechtsklick
            edge = self.get_edge_at(x, y)
            if edge:
                self.add_accident_to_edge(*edge, x, y)
            return
        
        if self.mode == "add_node":
            self.add_node(x, y)
        elif self.mode == "add_edge":
            clicked_node = self.get_node_at(x, y)
            if clicked_node is not None:
                if self.selected_node is None:
                    self.selected_node = clicked_node
                    self.highlight_node(clicked_node)
                else:
                    if clicked_node != self.selected_node:
                        self.add_edge(self.selected_node, clicked_node)
                    self.selected_node = None
                    self.draw_graph()
        elif self.mode == "set_start":
            clicked_node = self.get_node_at(x, y)
            if clicked_node is not None:
                self.start_node = clicked_node
                self.draw_graph()
        elif self.mode == "set_goal":
            clicked_node = self.get_node_at(x, y)
            if clicked_node is not None:
                self.goal_node = clicked_node
                self.draw_graph()
    
if __name__ == "__main__":
    root = tk.Tk()
    app = GraphTool(root)
    root.mainloop()

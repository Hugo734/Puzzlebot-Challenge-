#!/usr/bin/env python3
"""rack_editor.py — Coloca racks individuales y agrúpalos."""

import math, os, yaml
import tkinter as tk
from tkinter import messagebox, simpledialog

# ── Almacén ───────────────────────────────────────────────────────
OW, OH, THICK = 4.88, 3.69, 0.03

# Rack individual (full_rack.stl @ scale=0.01)
RACK_L = 0.35   # largo (eje principal)
RACK_P = 0.12   # profundidad

SAVE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'src', 'slam', 'config', 'rack_groups.yaml')

# Fallback: ruta absoluta conocida
if not os.path.isfile(SAVE_PATH):
    SAVE_PATH = os.path.expanduser(
        '~/Puzzlebot-Challenge-/src/slam/config/rack_groups.yaml')

CW, CH, MAR = 900, int(900 * OH / OW), 50

def w2c(wx, wy):
    px = MAR + wx / OW * (CW - 2*MAR)
    py = MAR + (1 - wy / OH) * (CH - 2*MAR)
    return px, py

def c2w(px, py):
    wx = (px - MAR) / (CW - 2*MAR) * OW
    wy = (1 - (py - MAR) / (CH - 2*MAR)) * OH
    return round(wx, 3), round(wy, 3)

def dpx2m(dpx, dpy):
    """Delta píxeles → delta metros."""
    dwx =  dpx / (CW - 2*MAR) * OW
    dwy = -dpy / (CH - 2*MAR) * OH
    return dwx, dwy

GROUP_COLORS = {
    None:   '#888888',
    'A':    '#CC3333', 'B': '#3355CC', 'C': '#229933',
    'D':    '#CC8800', 'E': '#AA33CC', 'F': '#33AACC',
    'G':    '#CC5533', 'H': '#33CC88', 'I': '#8833CC',
}

class RackEditor:
    def __init__(self, root):
        self.root = root
        root.title('Rack Editor — Puzzlebot AMR')
        root.configure(bg='#2B2B2B')

        self.racks   = []      # lista de dicts
        self.sel     = set()   # índices seleccionados
        self.tool    = tk.StringVar(value='place')
        self._drag_anchor = None   # (canvas_x, canvas_y) al inicio del drag
        self._drag_orig   = {}     # {idx: (cx_orig, cy_orig)}

        self._build_ui()
        self._load()
        self._redraw()

    # ─── UI ──────────────────────────────────────────────────────
    def _build_ui(self):
        self.root.columnconfigure(1, weight=1)

        left = tk.Frame(self.root, bg='#3C3F41', width=210, padx=6, pady=6)
        left.grid(row=0, column=0, sticky='ns')
        left.grid_propagate(False)

        def lbl(t, fg='#AAAAFF'):
            tk.Label(left, text=t, bg='#3C3F41', fg=fg,
                     font=('Courier', 9, 'bold')).pack(pady=(6,2))

        def btn(t, cmd, bg='#445566'):
            tk.Button(left, text=t, command=cmd, bg=bg, fg='white',
                      relief='flat', font=('Courier', 9), pady=3,
                      cursor='hand2').pack(fill='x', pady=1)

        lbl('HERRAMIENTA')
        for txt, val in [('📍 Colocar rack','place'),
                         ('👆 Seleccionar','select')]:
            tk.Radiobutton(left, text=txt, variable=self.tool, value=val,
                           bg='#3C3F41', fg='white', selectcolor='#556699',
                           activebackground='#3C3F41', font=('Courier',9),
                           cursor='hand2').pack(anchor='w')

        lbl('RACK (individual)')
        # Yaw del rack que se colocará
        yf = tk.Frame(left, bg='#3C3F41'); yf.pack(fill='x')
        tk.Label(yf, text='Yaw °:', bg='#3C3F41', fg='#CCCCCC',
                 font=('Courier',8), width=7, anchor='w').pack(side='left')
        self._yaw_var = tk.DoubleVar(value=90.0)
        tk.Spinbox(yf, textvariable=self._yaw_var, from_=-180, to=180,
                   increment=15, width=7, bg='#4A4A4A', fg='white',
                   buttonbackground='#556', font=('Courier',8),
                   relief='flat').pack(side='left')

        lbl('SELECCIÓN')
        btn('🔲 Selec. todos',   self._sel_all,    '#445566')
        btn('❌ Deselec.',        self._sel_none,   '#445566')
        btn('🗑  Borrar selec.',  self._delete_sel, '#774444')

        lbl('GRUPOS')
        btn('📦 Agrupar selec.', self._group_sel,  '#225544')
        btn('✂️  Desagrupar',     self._ungroup_sel,'#553322')

        lbl('VISTA')
        btn('👁  Vista 3D',       self._view_3d,    '#553388')

        lbl('ARCHIVO')
        btn('💾 Guardar YAML',   self._save,       '#225522')
        btn('📂 Cargar YAML',    self._load_dlg,   '#444444')

        # ── Info del rack seleccionado ─────────────────────────────
        lbl('INFO SELECCIONADO', fg='#AAAAAA')
        self._info_var = tk.StringVar(value='—')
        tk.Label(left, textvariable=self._info_var, bg='#3C3F41', fg='#CCCCCC',
                 font=('Courier', 7), justify='left', wraplength=190).pack(anchor='w')

        # Editar posición del seleccionado
        lbl('MOVER RACK', fg='#AAAAAA')
        fields = [
            ('cx',    -10,  10,   0.01),
            ('cy',    -10,  10,   0.01),
            ('yaw',  -180, 180,   5.0 ),
            ('largo', 0.01, 5.0,  0.01),
            ('prof',  0.01, 5.0,  0.01),
        ]
        for axis, mn, mx, inc in fields:
            rf = tk.Frame(left, bg='#3C3F41'); rf.pack(fill='x', pady=1)
            tk.Label(rf, text=f'{axis}:', bg='#3C3F41', fg='#CCCCCC',
                     font=('Courier',8), width=6, anchor='w').pack(side='left')
            var = tk.StringVar()
            sp = tk.Spinbox(rf, textvariable=var, from_=mn, to=mx, increment=inc,
                            width=8, bg='#4A4A4A', fg='white',
                            buttonbackground='#556', font=('Courier',8),
                            relief='flat')
            sp.pack(side='left')
            var.trace_add('write', lambda *a, ax=axis: self._edit_rack(ax))
            setattr(self, f'_var_{axis}', var)

        # ── Listbox de grupos ──────────────────────────────────────
        lbl('GRUPOS EXISTENTES', fg='#AAAAAA')
        self.lb = tk.Listbox(left, bg='#2B2B2B', fg='white',
                             font=('Courier',7), selectbackground='#4455AA',
                             relief='flat', height=8)
        self.lb.pack(fill='both', expand=True)
        self.lb.bind('<<ListboxSelect>>', self._lb_sel)

        # ── Canvas ────────────────────────────────────────────────
        rf2 = tk.Frame(self.root, bg='#2B2B2B')
        rf2.grid(row=0, column=1, sticky='nsew', padx=4, pady=4)

        self.cv = tk.Canvas(rf2, width=CW, height=CH,
                            bg='#F0EFE0', highlightthickness=1,
                            highlightbackground='#666')
        self.cv.pack()
        self.cv.bind('<Button-1>',         self._c_click)
        self.cv.bind('<B1-Motion>',        self._c_drag)
        self.cv.bind('<ButtonRelease-1>',  self._c_release)
        self.cv.bind('<Shift-Button-1>',   self._c_shift_click)

        self._stv = tk.StringVar(value='📍 Click = colocar rack | 👆 Seleccionar + arrastrar = mover')
        tk.Label(rf2, textvariable=self._stv, bg='#2B2B2B', fg='#AAAAAA',
                 font=('Courier',8)).pack()

    # ─── REDRAW ──────────────────────────────────────────────────
    def _redraw(self):
        self.cv.delete('all')

        # Suelo interior
        x0,y0=w2c(THICK,THICK); x1,y1=w2c(OW-THICK,OH-THICK)
        self.cv.create_rectangle(x0,y1,x1,y0, fill='#F5F5DC', outline='')

        # Paredes
        def prect(wx0,wy0,wx1,wy1):
            cx0,cy0=w2c(wx0,wy0); cx1,cy1=w2c(wx1,wy1)
            self.cv.create_rectangle(cx0,cy1,cx1,cy0, fill='#222', outline='')

        prect(0, OH-THICK, OW, OH)
        prect(0, 0,        OW, THICK)
        prect(OW-THICK, THICK, OW, OH-THICK)
        y=0.0
        for length, solid in [(0.35,True),(0.37,False),(0.15,True),(0.37,False),
                               (0.15,True),(0.37,False),(0.53,True),(0.80,False),(0.60,True)]:
            if solid: prect(0, y, THICK, y+length)
            y += length

        # Grid
        for xg in range(6):
            px,_=w2c(xg,0); _,ya=w2c(0,0); _,yb=w2c(0,OH)
            self.cv.create_line(px,ya,px,yb, fill='#CCCCAA',dash=(2,6),width=.5)
            self.cv.create_text(px,yb+10,text=f'{xg}',fill='#888',font=('Courier',7))
        for yg in range(5):
            _,py=w2c(0,yg); xa,_=w2c(0,0); xb,_=w2c(OW,0)
            self.cv.create_line(xa,py,xb,py, fill='#CCCCAA',dash=(2,6),width=.5)
            self.cv.create_text(xa-10,py,text=f'{yg}',fill='#888',font=('Courier',7))

        # Racks
        for i, r in enumerate(self.racks):
            selected = i in self.sel
            grp      = r.get('group')
            col      = GROUP_COLORS.get(grp, '#886600')
            yaw      = math.radians(r.get('yaw', 0.0))
            cx, cy   = r['cx'], r['cy']
            hl = r.get('largo', RACK_L) / 2
            hp = r.get('prof',  RACK_P) / 2
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            corners = [(-hl,-hp),(hl,-hp),(hl,hp),(-hl,hp)]
            pts = []
            for lx, ly in corners:
                wx = lx*cos_y - ly*sin_y + cx
                wy = lx*sin_y + ly*cos_y + cy
                px, py = w2c(wx, wy)
                pts += [px, py]

            outline = 'yellow' if selected else ('white' if grp else '#555')
            width   = 3 if selected else 1
            self.cv.create_polygon(pts, fill=col, outline=outline,
                                   width=width)

            # Flecha de dirección
            ax = math.cos(yaw)*hl*.8; ay = math.sin(yaw)*hl*.8
            px0,py0=w2c(cx,cy); px1,py1=w2c(cx+ax,cy+ay)
            self.cv.create_line(px0,py0,px1,py1,fill='white',width=1,
                                arrow=tk.LAST,arrowshape=(5,6,2))

            # Etiqueta grupo
            if grp:
                px,py=w2c(cx,cy)
                self.cv.create_text(px,py,text=grp,fill='white',
                                    font=('Courier',7,'bold'))

        # Leyenda grupos
        groups_seen = sorted({r.get('group') for r in self.racks if r.get('group')})
        for gi, g in enumerate(groups_seen):
            col = GROUP_COLORS.get(g,'#886600')
            self.cv.create_rectangle(CW-90, 10+gi*14, CW-76, 22+gi*14,
                                     fill=col, outline='white')
            self.cv.create_text(CW-72, 16+gi*14, text=f'Grupo {g}',
                                anchor='w', fill='white', font=('Courier',7))

        # Estado
        n_sel = len(self.sel)
        self.cv.create_text(4, CH-4, anchor='sw',
            text=f'{len(self.racks)} racks | {n_sel} selec. | {len(groups_seen)} grupos',
            fill='#888', font=('Courier',8))

        self._update_lb()
        self._update_info()

    # ─── CANVAS EVENTOS ──────────────────────────────────────────
    def _c_click(self, e):
        if self.tool.get() == 'place':
            wx, wy = c2w(e.x, e.y)
            try:    largo = float(self._var_largo.get())
            except: largo = RACK_L
            try:    prof  = float(self._var_prof.get())
            except: prof  = RACK_P
            self.racks.append({
                'cx': wx, 'cy': wy,
                'yaw':   float(self._yaw_var.get()),
                'largo': round(largo, 4),
                'prof':  round(prof,  4),
                'group': None,
            })
            self._redraw()
            return

        # Herramienta seleccionar
        hit = self._hit(e.x, e.y)
        if hit is not None:
            self.sel = {hit}
            self._drag_anchor = (e.x, e.y)
            self._drag_orig   = {hit: (self.racks[hit]['cx'],
                                       self.racks[hit]['cy'])}
        else:
            self.sel = set()
            self._drag_anchor = None
        self._redraw()

    def _c_shift_click(self, e):
        """Shift+click añade/quita de la selección."""
        if self.tool.get() != 'select': return
        hit = self._hit(e.x, e.y)
        if hit is not None:
            if hit in self.sel:
                self.sel.discard(hit)
            else:
                self.sel.add(hit)
                self._drag_anchor = None
        self._redraw()

    def _c_drag(self, e):
        if self.tool.get() != 'select': return
        if not self.sel or self._drag_anchor is None: return

        # Inicializar orig para todos los seleccionados al empezar drag
        if len(self._drag_orig) == 0 or list(self._drag_orig.keys()) != list(self.sel):
            self._drag_orig = {i: (self.racks[i]['cx'], self.racks[i]['cy'])
                               for i in self.sel}

        dpx = e.x - self._drag_anchor[0]
        dpy = e.y - self._drag_anchor[1]
        dwx, dwy = dpx2m(dpx, dpy)

        for i in self.sel:
            ox, oy = self._drag_orig[i]
            self.racks[i]['cx'] = round(ox + dwx, 3)
            self.racks[i]['cy'] = round(oy + dwy, 3)

        self._stv.set(f'↔ Moviendo {len(self.sel)} rack(s)  Δ=({dwx:.3f},{dwy:.3f})m')
        self._redraw()

    def _c_release(self, _):
        if self.sel and self._drag_anchor:
            self._drag_anchor = None
            self._stv.set('✔ Movido')
            self._redraw()

    def _hit(self, cpx, cpy):
        """Devuelve índice del rack más cercano al punto canvas, o None."""
        wx, wy = c2w(cpx, cpy)
        for i in reversed(range(len(self.racks))):
            r = self.racks[i]
            dx = wx - r['cx'];  dy = wy - r['cy']
            yaw = math.radians(r.get('yaw', 0.0))
            lx =  dx*math.cos(-yaw) - dy*math.sin(-yaw)
            ly =  dx*math.sin(-yaw) + dy*math.cos(-yaw)
            rl = r.get('largo', RACK_L); rp = r.get('prof', RACK_P)
            if abs(lx) <= rl/2 and abs(ly) <= rp/2:
                return i
        return None

    # ─── EDIT RACK DIRECTO ────────────────────────────────────────
    def _edit_rack(self, axis):
        if len(self.sel) != 1: return
        idx = next(iter(self.sel))
        var = getattr(self, f'_var_{axis}')
        try:
            val = float(var.get())
            self.racks[idx][axis] = round(val, 4)
            self._redraw()
        except (ValueError, tk.TclError):
            pass

    def _edit_all_sel(self, axis):
        """Aplica largo/prof a todos los seleccionados."""
        var = getattr(self, f'_var_{axis}')
        try:
            val = round(float(var.get()), 4)
        except (ValueError, tk.TclError):
            return
        for i in self.sel:
            self.racks[i][axis] = val
        self._redraw()

    # ─── SELECCIÓN ───────────────────────────────────────────────
    def _sel_all(self):
        self.sel = set(range(len(self.racks)))
        self._redraw()

    def _sel_none(self):
        self.sel = set()
        self._redraw()

    def _delete_sel(self):
        if not self.sel: return
        self.racks = [r for i,r in enumerate(self.racks) if i not in self.sel]
        self.sel = set()
        self._redraw()

    # ─── GRUPOS ──────────────────────────────────────────────────
    def _group_sel(self):
        if not self.sel:
            messagebox.showwarning('Sin selección', 'Selecciona racks primero.')
            return
        name = simpledialog.askstring('Nombre del grupo',
                                      'Letra o nombre del grupo (ej: A, B, C1):',
                                      parent=self.root)
        if not name: return
        name = name.strip().upper()
        for i in self.sel:
            self.racks[i]['group'] = name
        self._redraw()

    def _ungroup_sel(self):
        for i in self.sel:
            self.racks[i]['group'] = None
        self._redraw()

    # ─── LISTBOX ─────────────────────────────────────────────────
    def _update_lb(self):
        self.lb.delete(0,'end')
        groups = {}
        for r in self.racks:
            g = r.get('group') or '(sin grupo)'
            groups.setdefault(g, []).append(r)
        for g, rs in sorted(groups.items()):
            self.lb.insert('end', f'[{g}] — {len(rs)} racks')

    def _lb_sel(self, _=None):
        sel = self.lb.curselection()
        if not sel: return
        txt = self.lb.get(sel[0])
        # extraer nombre del grupo
        import re
        m = re.search(r'\[(.+?)\]', txt)
        if not m: return
        gname = m.group(1)
        if gname == '(sin grupo)': gname = None
        self.sel = {i for i,r in enumerate(self.racks)
                    if r.get('group') == gname}
        self._redraw()

    def _update_info(self):
        if len(self.sel) == 1:
            r = self.racks[next(iter(self.sel))]
            self._info_var.set(
                f"cx={r['cx']:.3f}  cy={r['cy']:.3f}\n"
                f"largo={r.get('largo',RACK_L):.3f}  prof={r.get('prof',RACK_P):.3f}\n"
                f"yaw={r.get('yaw',0):.1f}°  grupo={r.get('group','—')}")
            for ax in ('cx','cy','yaw','largo','prof'):
                var = getattr(self, f'_var_{ax}')
                default = RACK_L if ax=='largo' else (RACK_P if ax=='prof' else 0)
                try: var.set(f"{r.get(ax, default):.3f}")
                except tk.TclError: pass
        elif self.sel:
            self._info_var.set(f'{len(self.sel)} racks seleccionados')
        else:
            self._info_var.set('—')

    # ─── VISTA 3D ────────────────────────────────────────────────
    def _view_3d(self):
        try:
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        except ImportError:
            messagebox.showerror('Error', 'pip install matplotlib')
            return

        # ── Cargar STL ────────────────────────────────────────────
        stl_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'src','description','meshes','other','full_rack.stl')
        stl_tris = None
        if os.path.isfile(stl_path):
            import struct
            with open(stl_path,'rb') as f:
                f.read(80)
                n = struct.unpack('<I', f.read(4))[0]
                tris = []
                for _ in range(n):
                    f.read(12)
                    v0 = struct.unpack('<fff', f.read(12))
                    v1 = struct.unpack('<fff', f.read(12))
                    v2 = struct.unpack('<fff', f.read(12))
                    f.read(2)
                    tris.append((v0,v1,v2))
            stl_tris = tris

        # ── Figura ────────────────────────────────────────────────
        fig = plt.figure(figsize=(14,9), facecolor='#1A1A2E')
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#1A1A2E')
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False; pane.set_edgecolor('#444')

        def box_faces(x,y,z,dx,dy,dz):
            x1,y1,z1=x+dx,y+dy,z+dz
            return [[(x,y,z),(x1,y,z),(x1,y1,z),(x,y1,z)],
                    [(x,y,z1),(x1,y,z1),(x1,y1,z1),(x,y1,z1)],
                    [(x,y,z),(x1,y,z),(x1,y,z1),(x,y,z1)],
                    [(x,y1,z),(x1,y1,z),(x1,y1,z1),(x,y1,z1)],
                    [(x,y,z),(x,y1,z),(x,y1,z1),(x,y,z1)],
                    [(x1,y,z),(x1,y1,z),(x1,y1,z1),(x1,y,z1)]]

        def add_box(x,y,z,dx,dy,dz,color,alpha=0.9):
            pc = Poly3DCollection(box_faces(x,y,z,dx,dy,dz),
                                  linewidths=0.2, edgecolors='#333')
            pc.set_facecolor(color); pc.set_alpha(alpha)
            ax.add_collection3d(pc)

        # Suelo
        pc = Poly3DCollection([[(0,0,0),(OW,0,0),(OW,OH,0),(0,OH,0)]],alpha=0.15)
        pc.set_facecolor('#888'); ax.add_collection3d(pc)

        # ── Paredes del almacén ───────────────────────────────────
        WH = 0.41   # altura paredes
        add_box(0,       OH-THICK, 0, OW,   THICK,      WH, '#4A4A4A')
        add_box(0,       0,        0, OW,   THICK,      WH, '#4A4A4A')
        add_box(OW-THICK,THICK,    0, THICK,OH-2*THICK, WH, '#4A4A4A')
        # Pared Sur con segmentos
        segs=[(0.35,True),(0.37,False),(0.15,True),(0.37,False),
              (0.15,True),(0.37,False),(0.53,True),(0.80,False),(0.60,True)]
        y=0.0
        for length,solid in segs:
            if solid:
                add_box(0,y,0,THICK,length,WH,'#4A4A4A')
            else:   # dintel
                add_box(0,y,0.26,THICK,length,0.15,'#5A5A5A',alpha=0.6)
            y+=length

        # ── Racks ─────────────────────────────────────────────────
        SCALE = 0.01
        group_colors = {
            None:  '#886600',
            'A':   '#CC3333', 'B': '#3355CC', 'C': '#229933',
            'D':   '#CC8800', 'E': '#AA33CC', 'F': '#33AACC',
            'G':   '#CC5533', 'H': '#33CC88',
        }

        def add_rack_stl(cx, cy, yaw_rad, color, largo=RACK_L, prof=RACK_P, z_h=0.20):
            # STL nativo (scale=0.01): X=0.12m(prof), Y=0.35m(largo), Z=0.265m(alto)
            # Escalamos cada eje para que coincida exactamente con largo/prof/z_h
            STL_LARGO = 0.35    # Y nativo del STL
            STL_PROF  = 0.12    # X nativo del STL
            STL_ALTO  = 0.265   # Z nativo del STL
            scale_l = largo / STL_LARGO
            scale_p = prof  / STL_PROF
            scale_z = z_h   / STL_ALTO

            if stl_tris is None:
                add_box(cx-largo/2, cy-prof/2, 0, largo, prof, z_h, color)
                return
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
            faces = []
            for (v0,v1,v2) in stl_tris:
                verts = []
                for (vx,vy,vz) in (v0,v1,v2):
                    sx = vx*SCALE + 6*SCALE       # centrar X del STL
                    sy = vy*SCALE + 13.5*SCALE    # centrar Y del STL
                    lz = vz*SCALE * scale_z       # altura exacta
                    # Pre-rotar -90° + escalar largo y prof
                    lx =  sy * scale_l
                    ly = -sx * scale_p
                    rx = lx*cos_y - ly*sin_y + cx
                    ry = lx*sin_y + ly*cos_y + cy
                    verts.append((rx, ry, lz))
                faces.append(verts)
            # Renderizar en chunks
            for i in range(0, len(faces), 300):
                pc = Poly3DCollection(faces[i:i+300],
                                      linewidths=0.0, edgecolors='none')
                pc.set_facecolor(color); pc.set_alpha(0.92)
                ax.add_collection3d(pc)

        # Leer z del YAML (mismo para todos los racks)
        rack_z = 0.20   # default
        yaml_path = os.path.expanduser(SAVE_PATH)
        if os.path.isfile(yaml_path):
            import yaml as _yaml
            with open(yaml_path) as _f:
                _d = _yaml.safe_load(_f) or {}
            rack_z = float((_d.get('rack_dims') or {}).get('z', 0.20))

        for r in self.racks:
            grp   = r.get('group')
            color = group_colors.get(grp, '#886600')
            yaw   = math.radians(r.get('yaw', 0.0))
            add_rack_stl(r['cx'], r['cy'], yaw, color,
                         largo=r.get('largo', RACK_L),
                         prof =r.get('prof',  RACK_P),
                         z_h  =rack_z)

        # ── Ejes con proporciones reales ──────────────────────────
        ax.set_xlim(0, OW)
        ax.set_ylim(0, OH)
        ax.set_zlim(0, WH + 0.05)
        # Proporciones físicas reales: X=4.88m, Y=3.69m, Z=0.41m
        ax.set_box_aspect([OW, OH, WH])
        ax.set_xlabel('X (m)', color='#AAA', labelpad=4)
        ax.set_ylabel('Y (m)', color='#AAA', labelpad=4)
        ax.set_zlabel('Z (m)', color='#AAA', labelpad=4)
        ax.tick_params(colors='#AAA', labelsize=7)

        groups_seen = sorted({r.get('group') for r in self.racks if r.get('group')})
        handles = [plt.Rectangle((0,0),1,1,
                   color=group_colors.get(g,'#886600'), label=f'Grupo {g}')
                   for g in groups_seen]
        if handles:
            ax.legend(handles=handles, loc='upper left',
                      facecolor='#222', labelcolor='white', fontsize=8)

        ax.set_title(f'Vista 3D — {len(self.racks)} racks  |  '
                     f'{len(groups_seen)} grupos\n'
                     '(arrastra para rotar · rueda para zoom)',
                     color='white', fontsize=10)
        plt.tight_layout()
        plt.show()

    # ─── GUARDAR / CARGAR ────────────────────────────────────────
    def _save(self):
        os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
        # Agrupar para el YAML
        groups = {}
        ungrouped = []
        for r in self.racks:
            g = r.get('group')
            if g:
                groups.setdefault(g, []).append(
                    {'cx': r['cx'], 'cy': r['cy'], 'yaw': r.get('yaw',0)})
            else:
                ungrouped.append(
                    {'cx': r['cx'], 'cy': r['cy'], 'yaw': r.get('yaw',0)})

        data = {
            'rack_dims': {'largo': RACK_L, 'prof': RACK_P, 'z': 0.265},
            'groups': {k: v for k,v in sorted(groups.items())},
        }
        if ungrouped:
            data['ungrouped'] = ungrouped

        with open(SAVE_PATH,'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        messagebox.showinfo('Guardado', f'Guardado:\n{SAVE_PATH}')

    def _load_dlg(self):
        self._load()

    def _load(self):
        path = os.path.expanduser(SAVE_PATH)
        if not os.path.isfile(path):
            messagebox.showwarning('No encontrado', f'No existe:\n{path}')
            return
        with open(path) as f:
            data = yaml.safe_load(f) or {}

        dims  = data.get('rack_dims', {})
        largo = float(dims.get('largo', RACK_L))
        prof  = float(dims.get('prof',  RACK_P))

        self.racks = []
        for gname, rs in (data.get('groups') or {}).items():
            for r in (rs or []):
                self.racks.append({
                    'cx':    float(r['cx']),
                    'cy':    float(r['cy']),
                    'yaw':   float(r.get('yaw', 0)),
                    'largo': float(r.get('largo', largo)),
                    'prof':  float(r.get('prof',  prof)),
                    'group': gname,
                })
        for r in (data.get('ungrouped') or []):
            self.racks.append({
                'cx':    float(r['cx']),
                'cy':    float(r['cy']),
                'yaw':   float(r.get('yaw', 0)),
                'largo': float(r.get('largo', largo)),
                'prof':  float(r.get('prof',  prof)),
                'group': None,
            })

        self.sel = set()
        self._update_lb()
        self._redraw()
        self._stv.set(f'✔ Cargado: {len(self.racks)} racks desde {os.path.basename(path)}')


def main():
    root = tk.Tk()
    RackEditor(root)
    root.mainloop()

if __name__ == '__main__':
    main()

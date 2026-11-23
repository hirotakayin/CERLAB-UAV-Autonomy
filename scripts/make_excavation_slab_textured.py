#!/usr/bin/env python3
"""Build a photo-textured, cluttered version of the excavation slab world, for testing monocular / panoramic depth
(DAP, DA3) on the 360 camera. Same geometry and collisions as models/excavation_slab (the pit, its deck on six columns
and the two short walls under it), so the exploration presets for map:=excavation_slab still apply.

  models/excavation_slab_textured/   the slab's boxes (collision + plain visuals, which also cast the shadows) and, on
                                     every surface a camera can see, a textured quad (OBJ, explicit UVs) a few mm in front
  worlds/excavation_slab_textured/excavation_slab_textured.world   shadows, a clouded sky, a low sun plus a sky fill
                                     light, the model above and seeded clutter (props from osrf/gazebo_models)

  make_excavation_slab_textured.py --textures <dir> [--seed 3] [--ppm 128] [--clutter 1.0]

--textures is a folder of ambientCG (CC0) 1K-JPG sets, one sub-folder each: <Id>/<Id>_1K-JPG_Color.jpg, _NormalGL.jpg and,
when the set has one, _AmbientOcclusion.jpg. Needed: Concrete046 Concrete034 Concrete036 Concrete031 Concrete042A Ground054
Ground037 Gravel022 (https://ambientcg.com/get?file=<Id>_1K-JPG.zip).

Gazebo Classic has no global illumination or normal mapping in material scripts, so the lighting is baked into each
surface's own texture instead (composed per surface at --ppm pixels per metre, so nothing is stretched or visibly tiled):
relief from the normal map under a fixed light, the set's ambient occlusion, occlusion where walls meet the floor and
each other and under the deck, formwork panel seams and tie holes, rain streaks down the walls, splash at their feet,
gravel patches, wheel tracks and puddles on the floor, dirt along the pit edge.
"""
import argparse
import os
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.normpath(os.path.join(HERE, '..'))
NAME = 'excavation_slab_textured'
MODEL = os.path.join(SIM, 'models', NAME)
SRC_SDF = os.path.join(SIM, 'models', 'excavation_slab', 'excavation_slab.sdf')

# the pit (from excavation_slab.sdf): inner faces x 0..10, y 5..35, floor z -10, ground z 0; deck z -6.5..-6.2, y 12..28
PX0, PX1, PY0, PY1, FLOOR, TOP = 0.0, 10.0, 5.0, 35.0, -10.0, 0.0
DECK_Y0, DECK_Y1, DECK_Z0, DECK_Z1 = 12.0, 28.0, -6.5, -6.2
COLUMNS = [(x, y) for y in (16.0, 20.0, 24.0) for x in (3.3, 6.7)]
COL_HALF = 0.2
SLAB_WALLS = [((0.0, 6.5), 15.0), ((3.5, 10.0), 25.0)]  # (x range, y centre), 0.4 thick, floor to deck
EPS = 0.004                                            # quads sit this far in front of their box face


# ------------------------------------------------------------------ texture sets
def fractal(rng, h, w, octaves=5, persistence=0.55, base=3):
    """Smooth value noise in [0, 1], h x w."""
    out, amp, total = np.zeros((h, w)), 1.0, 0.0
    for o in range(octaves):
        n = base * 2 ** o
        layer = rng.random((max(2, int(n * h / max(h, w))) + 1, max(2, int(n * w / max(h, w))) + 1))
        out += amp * np.asarray(Image.fromarray((layer * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC), float) / 255
        total += amp; amp *= persistence
    out /= total
    return (out - out.min()) / max(out.max() - out.min(), 1e-6)


class TexSet:
    """One ambientCG set as a lit tile: albedo x its AO x relief from its normal map under a fixed light."""
    LIGHT = np.array([0.35, 0.45, 0.82]) / np.linalg.norm([0.35, 0.45, 0.82])

    def __init__(self, folder, set_id, tile_m, relief=0.55):
        p = lambda k: os.path.join(folder, set_id, f'{set_id}_1K-JPG_{k}.jpg')
        col = np.asarray(Image.open(p('Color')).convert('RGB'), float) / 255
        nrm = np.asarray(Image.open(p('NormalGL')).convert('RGB'), float) / 255 * 2 - 1
        shade = np.clip((nrm @ self.LIGHT) / self.LIGHT[2], 0.4, 1.6)
        col = col * (1 + relief * (shade - 1))[..., None]
        if os.path.exists(p('AmbientOcclusion')):
            col = col * (np.asarray(Image.open(p('AmbientOcclusion')).convert('L'), float) / 255)[..., None] ** 0.7
        self.tile, self.tile_m = np.clip(col, 0, 1), tile_m

    def canvas(self, rng, h, w, ppm):
        """The set tiled over h x w pixels at ppm (random offset)."""
        n = max(8, int(round(self.tile_m * ppm)))
        t = np.asarray(Image.fromarray((self.tile * 255).astype(np.uint8)).resize((n, n), Image.LANCZOS), float) / 255
        big = np.tile(t, (h // n + 2, w // n + 2, 1))
        oy, ox = rng.integers(0, n, 2)
        return big[oy:oy + h, ox:ox + w]


def mix(a, b, mask):
    return a * (1 - mask[..., None]) + b * mask[..., None]


def soft_mask(rng, h, w, coverage, sharp=14.0, base=3):
    n = fractal(rng, h, w, base=base)
    thr = np.quantile(n, 1 - coverage)
    return 1 / (1 + np.exp(-sharp * (n - thr)))


def tone(img, rng, amount=0.12, base=2):
    return img * (1 + amount * (fractal(rng, *img.shape[:2], base=base) - 0.5) * 2)[..., None]


# ------------------------------------------------------------------ surfaces
def wall_texture(S, rng, w_m, h_m, ppm, sets=('Concrete034', 'Concrete046'), stain='Concrete036', feet=True, top_lip=True,
                 panel=(2.4, 1.2), ties=True, streaks=True, corner_ao=True):
    """A cast-in-place wall seen from the front, image top = wall top."""
    h, w = int(h_m * ppm), int(w_m * ppm)
    img = mix(S[sets[0]].canvas(rng, h, w, ppm), S[sets[1]].canvas(rng, h, w, ppm), soft_mask(rng, h, w, 0.4, base=6) * 0.35)
    img = mix(img, S[stain].canvas(rng, h, w, ppm) * 0.95, soft_mask(rng, h, w, 0.2, base=8) * 0.3)
    img = tone(img, rng, 0.10)
    yy, xx = np.mgrid[0:h, 0:w] / ppm                       # metres from the top-left corner
    if panel:                                                # formwork panels: each pour a slightly different tone
        pw, ph = panel
        pid = (xx // pw).astype(int) * 1000 + (yy // ph).astype(int)
        lut = rng.normal(1, 0.035, pid.max() + 1)
        img *= lut[pid][..., None]
        seam = (np.minimum(xx % pw, pw - xx % pw) < 0.008) | (np.minimum(yy % ph, ph - yy % ph) < 0.006)
        img[seam] *= 0.86
    if ties:                                                 # tie holes on a 0.6 m grid inside each panel
        im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)); d = ImageDraw.Draw(im)
        for gx in np.arange(0.3, w_m, 0.6):
            for gy in np.arange(0.3, h_m, 0.6):
                x, y, r = gx * ppm, gy * ppm, 0.014 * ppm
                d.ellipse([x - r, y - r, x + r, y + r], fill=(58, 56, 53))
        img = np.asarray(im, float) / 255
    if streaks:                                              # rain streaks from the top edge, fading down
        st = np.zeros((h, w))
        for _ in range(int(w_m * 1.2)):
            x0, wid, length = rng.uniform(0, w_m), rng.uniform(0.04, 0.35), rng.uniform(0.6, min(5.0, h_m))
            prof = np.exp(-((xx[0] - x0) / wid) ** 2)
            fade = np.clip(1 - yy[:, 0] / length, 0, 1) ** 1.5
            st = np.maximum(st, fade[:, None] * prof[None, :] * rng.uniform(0.4, 1.0))
        st *= 0.6 + 0.4 * fractal(rng, h, w, base=12)
        img = img * (1 - 0.45 * st)[..., None] * np.array([1.0, 0.98, 0.93])[None, None, :] ** st[..., None]
    if feet:                                                 # occlusion and mud splash where the wall meets the floor
        hb = h_m - yy
        img *= (1 - 0.38 * np.exp(-hb / 0.55))[..., None]
        splash = np.clip(1 - hb / (0.25 + 0.35 * fractal(rng, h, w, base=20)), 0, 1)
        img = mix(img, img * np.array([0.62, 0.52, 0.40]), splash * 0.8)
    if corner_ao:
        dc = np.minimum(xx, w_m - xx)
        img *= (1 - 0.22 * np.exp(-dc / 0.45))[..., None]
    if top_lip:
        img *= (1 + 0.08 * np.exp(-yy / 0.15))[..., None]
    return np.clip(img, 0, 1)


def pit_floor_texture(S, rng, w_m, h_m, ppm):
    """The pit floor seen from above: image right = +x, image top = +y (far end)."""
    h, w = int(h_m * ppm), int(w_m * ppm)
    img = S['Ground054'].canvas(rng, h, w, ppm)
    img = mix(img, S['Gravel022'].canvas(rng, h, w, ppm), soft_mask(rng, h, w, 0.3, sharp=10, base=5) * 0.8)
    img = tone(img, rng, 0.14)
    yy, xx = np.mgrid[0:h, 0:w] / ppm
    y_world = PY1 - yy; x_world = PX0 + xx
    # two wheel tracks along the pit, gently curving, compacted and darker
    for off in (-0.9, 0.9):
        centre = 5.0 + 1.6 * np.sin((y_world - 5) / 30 * 2.2 * np.pi + 0.6) + off
        band = np.exp(-((x_world - centre) / 0.22) ** 2) * (0.7 + 0.3 * fractal(rng, h, w, base=30))
        img *= (1 - 0.18 * band)[..., None]
    # puddles: darker, smoother, a little blue from the sky
    pud = soft_mask(rng, h, w, 0.015, sharp=60, base=10)
    smooth = np.asarray(Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(ppm * 0.05)), float) / 255
    img = mix(img, smooth * np.array([0.62, 0.64, 0.68]), pud * 0.75)
    # occlusion along the pit walls and under the deck, and around the columns and short walls
    dwall = np.minimum.reduce([x_world - PX0, PX1 - x_world, y_world - PY0, PY1 - y_world])
    ao = 1 - 0.35 * np.exp(-dwall / 0.7)
    under = 1 / (1 + np.exp(-(np.minimum(y_world - DECK_Y0, DECK_Y1 - y_world)) / 0.6))
    ao *= 1 - 0.22 * under
    for cx, cy in COLUMNS:
        d = np.maximum(np.abs(x_world - cx), np.abs(y_world - cy)) - COL_HALF
        ao *= 1 - 0.35 * np.exp(-np.maximum(d, 0) / 0.25)
    for (xa, xb), yc in SLAB_WALLS:
        d = np.maximum(np.maximum(xa - x_world, x_world - xb), np.abs(y_world - yc) - 0.2)
        ao *= 1 - 0.3 * np.exp(-np.maximum(d, 0) / 0.3)
    return np.clip(img * ao[..., None], 0, 1)


def ground_texture(S, rng, w_m, h_m, ppm, x0, y1, apron=False):
    """Ground around the pit from above (image right = +x, top = +y): grass and dirt, bare and darker near the edge."""
    h, w = int(h_m * ppm), int(w_m * ppm)
    img = mix(S['Ground054'].canvas(rng, h, w, ppm), S['Ground037'].canvas(rng, h, w, ppm), soft_mask(rng, h, w, 0.55, sharp=8, base=5))
    if apron: img = mix(img, S['Gravel022'].canvas(rng, h, w, ppm), soft_mask(rng, h, w, 0.6, sharp=8, base=5))
    img = tone(img, rng, 0.12)
    yy, xx = np.mgrid[0:h, 0:w] / ppm
    xw, yw = x0 + xx, y1 - yy
    dx = np.maximum(np.maximum(PX0 - xw, xw - PX1), 0); dy = np.maximum(np.maximum(PY0 - yw, yw - PY1), 0)
    dedge = np.hypot(dx, dy)
    dirt = np.exp(-dedge / 1.5)                              # trampled, spoil-covered edge
    img = mix(img, S['Ground054'].canvas(rng, h, w, ppm) * 0.85, dirt * 0.8)
    return np.clip(img, 0, 1)


def deck_texture(S, rng, w_m, h_m, ppm, underside=False, joints=True):
    h, w = int(h_m * ppm), int(w_m * ppm)
    img = mix(S['Concrete031'].canvas(rng, h, w, ppm), S['Concrete042A'].canvas(rng, h, w, ppm), soft_mask(rng, h, w, 0.5, sharp=8, base=5))
    img = tone(img, rng, 0.08)
    yy, xx = np.mgrid[0:h, 0:w] / ppm
    if joints:
        j = (np.minimum(xx % 4.0, 4.0 - xx % 4.0) < 0.01) | (np.minimum(yy % 4.0, 4.0 - yy % 4.0) < 0.01)
        img[j] *= 0.6
    if underside:
        img *= 0.85
        pw = 1.2                                             # plywood formwork prints on the soffit
        img *= (1 + rng.normal(0, 0.03, (int(h_m / pw) + 2, int(w_m / 2.4) + 2))[(yy // pw).astype(int), (xx // 2.4).astype(int)])[..., None]
    return np.clip(img, 0, 1)


# ------------------------------------------------------------------ geometry: textured quads as OBJ
def quad_obj(path, mtl_name, tex_name, p0, eu, ev, w, h, normal, repeat=(1, 1)):
    """One quad: p0 = bottom-left as seen from the front, eu = right, ev = up (unit), front face along normal; the texture
    repeats repeat (u, v) times across it."""
    p0, eu, ev, normal = (np.asarray(v, float) for v in (p0, eu, ev, normal))
    if np.dot(np.cross(eu, ev), normal) < 0: raise ValueError('quad faces the wrong way')
    p0 = p0 + EPS * normal
    v = [p0, p0 + eu * w, p0 + eu * w + ev * h, p0 + ev * h]
    base = os.path.splitext(os.path.basename(path))[0]
    with open(path, 'w') as f:
        f.write(f'mtllib {base}.mtl\no {base}\n')
        for p in v: f.write('v %.4f %.4f %.4f\n' % tuple(p))
        ru, rv = repeat
        for t in ((0, 0), (ru, 0), (ru, rv), (0, rv)): f.write('vt %.3f %.3f\n' % t)
        f.write('vn %.4f %.4f %.4f\nusemtl %s\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n' % (*normal, mtl_name))
    with open(os.path.join(os.path.dirname(path), base + '.mtl'), 'w') as f:
        f.write(f'newmtl {mtl_name}\nKa 0.85 0.85 0.85\nKd 1.0 1.0 1.0\nKs 0.03 0.03 0.03\nNs 8\nd 1.0\nillum 2\nmap_Kd {tex_name}\n')


def save_tex(img, path):
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path, quality=90)


def surfaces(S, rng, ppm):
    """(name, image, p0, eu, ev, w, h, normal, repeat) for every surface a camera can see."""
    X, Y, Z = np.eye(3)
    H = TOP - FLOOR
    out = []
    lp = ppm
    # the four pit walls (inner faces), floor to ground
    # (seen from the front, right = up x normal)
    out.append(('wall_left', wall_texture(S, rng, PY1 - PY0, H, lp), (PX0, PY0, FLOOR), Y, Z, PY1 - PY0, H, X))
    out.append(('wall_right', wall_texture(S, rng, PY1 - PY0, H, lp), (PX1, PY1, FLOOR), -Y, Z, PY1 - PY0, H, -X))
    out.append(('wall_near', wall_texture(S, rng, PX1 - PX0, H, lp), (PX1, PY0, FLOOR), -X, Z, PX1 - PX0, H, Y))
    out.append(('wall_far', wall_texture(S, rng, PX1 - PX0, H, lp), (PX0, PY1, FLOOR), X, Z, PX1 - PX0, H, -Y))
    # the pit floor
    out.append(('pit_floor', pit_floor_texture(S, rng, PX1 - PX0, PY1 - PY0, lp), (PX0, PY0, FLOOR), X, Y, PX1 - PX0, PY1 - PY0, Z))
    # the ground around (coarser: mostly seen far away)
    gp = ppm // 2
    for name, (x0, x1, y0, y1), apron in (('ground_left', (-10, 0, -5, 45), False), ('ground_right', (10, 20, -5, 45), False),
                                          ('ground_near', (0, 10, -5, 5), True), ('ground_far', (0, 10, 35, 45), True)):
        out.append((name, ground_texture(S, rng, x1 - x0, y1 - y0, gp, x0, y1, apron), (x0, y0, TOP), X, Y, x1 - x0, y1 - y0, Z))
    # the deck: top, soffit, and its two exposed edges
    dw, dl = PX1 - PX0, DECK_Y1 - DECK_Y0
    out.append(('deck_top', deck_texture(S, rng, dw, dl, lp), (PX0, DECK_Y0, DECK_Z1), X, Y, dw, dl, Z))
    out.append(('deck_soffit', deck_texture(S, rng, dw, dl, lp, underside=True, joints=False), (PX1, DECK_Y0, DECK_Z0), -X, Y, dw, dl, -Z))
    t = DECK_Z1 - DECK_Z0
    edge = lambda: wall_texture(S, rng, dw, t, lp, sets=('Concrete031', 'Concrete042A'), feet=False, top_lip=False, panel=None,
                                ties=False, streaks=False, corner_ao=False)
    out.append(('deck_edge_near', edge(), (PX0, DECK_Y0, DECK_Z0), X, Z, dw, t, -Y))
    out.append(('deck_edge_far', edge(), (PX1, DECK_Y1, DECK_Z0), -X, Z, dw, t, Y))
    # columns (four faces each) and the short walls under the deck (both faces and ends)
    ch = DECK_Z0 - FLOOR
    col = lambda w_m: wall_texture(S, rng, w_m, ch, lp, sets=('Concrete042A', 'Concrete031'), panel=(w_m, 1.2), ties=False,
                                   streaks=False, corner_ao=False)
    for i, (cx, cy) in enumerate(COLUMNS):
        a = COL_HALF
        for k, (p0, eu, n) in enumerate((((cx - a, cy - a, FLOOR), X, -Y), ((cx + a, cy - a, FLOOR), Y, X),
                                         ((cx + a, cy + a, FLOOR), -X, Y), ((cx - a, cy + a, FLOOR), -Y, -X))):
            out.append((f'column{i}_{k}', col(2 * a), p0, eu, Z, 2 * a, ch, n))
    for i, ((xa, xb), yc) in enumerate(SLAB_WALLS):
        wl = xb - xa
        sw = lambda: wall_texture(S, rng, wl, ch, lp, sets=('Concrete042A', 'Concrete031'), stain='Concrete036', streaks=False)
        out.append((f'slabwall{i}_s', sw(), (xa, yc - 0.2, FLOOR), X, Z, wl, ch, -Y))
        out.append((f'slabwall{i}_n', sw(), (xb, yc + 0.2, FLOOR), -X, Z, wl, ch, Y))
        for k, (x, eu, n) in enumerate(((xa, -Y, -X), (xb, Y, X))):
            if x in (PX0, PX1): continue                     # this end is cast into the pit wall
            y0 = yc + 0.2 if k == 0 else yc - 0.2
            out.append((f'slabwall{i}_end{k}', col(0.4), (x, y0, FLOOR), eu, Z, 0.4, ch, n))
    # the land beyond, to the horizon (~200 m): one 32 m tile repeated, a little darker and greener
    R, tile = 200.0, 32.0
    gx0, gx1, gy0, gy1 = -10.0, 20.0, -5.0, 45.0
    for name, (x0, x1, y0, y1) in (('outer_west', (gx0 - R, gx0, gy0 - R, gy1 + R)), ('outer_east', (gx1, gx1 + R, gy0 - R, gy1 + R)),
                                   ('outer_south', (gx0, gx1, gy0 - R, gy0)), ('outer_north', (gx0, gx1, gy1, gy1 + R))):
        n = int(tile * 32)
        img = mix(S['Ground037'].canvas(rng, n, n, 32), S['Ground054'].canvas(rng, n, n, 32), soft_mask(rng, n, n, 0.35, sharp=8, base=4) * 0.8)
        out.append((name, np.clip(img * 0.92, 0, 1), (x0, y0, TOP - 0.01), X, Y, x1 - x0, y1 - y0, Z, ((x1 - x0) / tile, (y1 - y0) / tile)))
    return [o if len(o) == 9 else o + ((1, 1),) for o in out]


# ------------------------------------------------------------------ clutter
PROPS = {  # model: (footprint radius m, on pit floor, on ground)
    'construction_cone': 0.25, 'construction_barrel': 0.35, 'euro_pallet': 0.75, 'cinder_block': 0.25,
    'drc_practice_4x4x20': 0.35, 'drc_practice_2x6': 0.7, 'dumpster': 1.3, 'utility_cart': 0.8, 'jersey_barrier': 1.1,
    'pickup': 2.8, 'person': 0.4,
}


def place_clutter(rng, scale):
    """Seeded props: (model, x, y, z, yaw). Pit floor at FLOOR, ground at TOP. Kept off walls, columns and each other."""
    placed = []
    blocked = [(cx, cy, COL_HALF + 0.3) for cx, cy in COLUMNS] + [(5.0, 2.0, 2.5)]   # + the drone's spawn (start_excavation_slab)
    for (xa, xb), yc in SLAB_WALLS:
        for x in np.arange(xa, xb + 0.01, 0.5): blocked.append((x, yc, 0.5))

    def free(x, y, r):
        return all(np.hypot(x - bx, y - by) > r + br for bx, by, br in blocked + [(p[1], p[2], PROPS[p[0]]) for p in placed])

    def put(model, region, z, yaw=None, tries=200):
        r = PROPS[model]
        for _ in range(tries):
            x, y = rng.uniform(region[0] + r, region[1] - r), rng.uniform(region[2] + r, region[3] - r)
            if free(x, y, r):
                placed.append((model, x, y, z, rng.uniform(-np.pi, np.pi) if yaw is None else yaw)); return True
        return False

    pit = (PX0 + 0.4, PX1 - 0.4, PY0 + 0.4, PY1 - 0.4)
    n = lambda k: max(1, int(round(k * scale)))
    # pit floor: a materials yard
    for _ in range(n(3)):                                    # pallets, some with blocks on them
        if put('euro_pallet', pit, FLOOR) and rng.random() < 0.7:
            _, x, y, _, yaw = placed[-1]
            for k in range(int(rng.integers(2, 6))):
                dx, dy = (k % 3 - 1) * 0.42, (k // 3 - 0.5) * 0.22
                c, s = np.cos(yaw), np.sin(yaw)
                placed.append(('cinder_block', x + c * dx - s * dy, y + s * dx + c * dy, FLOOR + 0.144, yaw))
    for _ in range(n(4)): put('construction_barrel', pit, FLOOR)
    for _ in range(n(2)):                                    # lumber stacks
        if put('drc_practice_4x4x20', pit, FLOOR):
            _, x, y, _, yaw = placed[-1]
            for k in range(1, int(rng.integers(2, 5))):
                placed.append(('drc_practice_4x4x20', x + rng.normal(0, 0.02), y + 0.1 * k, FLOOR, yaw))
    for _ in range(n(2)): put('drc_practice_2x6', pit, FLOOR)
    for _ in range(n(1)): put('utility_cart', pit, FLOOR)
    for _ in range(n(2)): put('person', pit, FLOOR)
    # a line of cones across the pit at the far end of the deck
    yline = DECK_Y1 + 1.5
    for x in np.arange(1.0, 9.5, 1.4):
        if free(x, yline, 0.25): placed.append(('construction_cone', x, yline, FLOOR, 0.0))
    # ground: jersey barriers along both long edges, a pickup, a dumpster, pallets, cones, a person
    for side, x in ((-1, PX0 - 1.4), (1, PX1 + 1.4)):
        for y in np.arange(PY0 + 2, PY1 - 1, 4.2 if scale >= 1 else 8.4):
            placed.append(('jersey_barrier', x, y, TOP, np.pi / 2))
    put('pickup', (-9.0, -3.5, PY0, PY1), TOP, yaw=np.pi / 2 + rng.normal(0, 0.1))
    put('dumpster', (13.0, 19.0, PY0, PY1), TOP, yaw=np.pi / 2)
    for _ in range(n(3)): put('euro_pallet', (12.5, 19.5, -4.0, 45.0), TOP)
    for _ in range(n(6)): put('construction_cone', (-1.0, 11.0, -4.5, 4.0), TOP)
    for _ in range(n(1)): put('person', (-3.0, 13.0, -4.0, 4.0), TOP)
    return placed


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--textures', required=True, help='folder of ambientCG 1K-JPG sets, one sub-folder per set id')
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--ppm', type=int, default=128, help='texture pixels per metre in the pit (the ground gets half)')
    ap.add_argument('--clutter', type=float, default=1.0, help='scale the number of props (0 = none)')
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    tiles = {'Concrete046': 2.5, 'Concrete034': 2.5, 'Concrete036': 2.0, 'Concrete031': 2.0, 'Concrete042A': 2.0,
             'Ground054': 2.0, 'Ground037': 2.0, 'Gravel022': 1.0}
    S = {k: TexSet(os.path.expanduser(a.textures), k, m) for k, m in tiles.items()}

    mesh_dir = os.path.join(MODEL, 'meshes'); os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir): os.remove(os.path.join(mesh_dir, f))

    # the original slab: keep its collisions and plain visuals (they close the edges and cast the shadows)
    src = ET.parse(SRC_SDF).getroot().find('model/link')
    parts = [ET.tostring(c, encoding='unicode') for c in src if c.tag in ('collision', 'visual')]
    for name, img, p0, eu, ev, w, h, n, rep in surfaces(S, rng, a.ppm):
        save_tex(img, os.path.join(mesh_dir, f'{name}.jpg'))
        quad_obj(os.path.join(mesh_dir, f'{name}.obj'), f'{NAME}_{name}', f'{name}.jpg', p0, eu, ev, w, h, n, rep)
        parts.append(f'<visual name="tex_{name}"><geometry><mesh><uri>model://{NAME}/meshes/{name}.obj</uri></mesh></geometry>'
                     f'<cast_shadows>false</cast_shadows></visual>')
    sdf = ('<?xml version="1.0"?>\n<!-- generated by uav_simulator/scripts/make_excavation_slab_textured.py: rerun it instead of editing -->\n'
           f'<sdf version="1.7"><model name="{NAME}"><static>true</static><link name="excavation_link">\n' + '\n'.join(parts) +
           '\n</link></model></sdf>\n')
    open(os.path.join(MODEL, f'{NAME}.sdf'), 'w').write(sdf)
    open(os.path.join(MODEL, 'model.config'), 'w').write(
        f'<?xml version="1.0"?>\n<model><name>{NAME}</name><version>1.0</version><sdf version="1.7">{NAME}.sdf</sdf>\n'
        f'<description>excavation_slab (same geometry and collisions) with photo textures and baked lighting, seed {a.seed}. '
        'Textures from ambientCG (CC0).</description></model>\n')

    # the world: lighting and sky for shadows, the model, clutter
    sun_dir = np.array([-0.45, 0.30, -0.84]); sun_dir /= np.linalg.norm(sun_dir)
    fill_dir = np.array([0.5, -0.4, -0.77]); fill_dir /= np.linalg.norm(fill_dir)
    props = place_clutter(rng, a.clutter) if a.clutter > 0 else []
    # people are named as in the other worlds (personN_<x>_<y>_<z> size): onboard_detector's fake detector reads every
    # model named person* / obstacle* as an obstacle and parses its size from the name's last 11 characters
    # The people also go first in the world: fakeDetector::stateCB indexes its per-target arrays (one entry per target)
    # with the target's index among ALL Gazebo models, so it reads past their end (segfault) unless the targets are
    # models 0..n-1 (as in the corridor worlds).
    label = lambda m, i: f'person{i}_0.5_0.5_1.8' if m == 'person' else f'{m}_{i}'
    props = [p for p in props if p[0] == 'person'] + [p for p in props if p[0] != 'person']
    inc = lambda keep: '\n'.join(f'    <include><name>{label(m, i)}</name><uri>model://{m}</uri><static>true</static>'
                                 f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose></include>'
                                 for i, (m, x, y, z, yaw) in enumerate(props) if keep(m))
    world = f'''<?xml version="1.0" ?>
<!-- generated by uav_simulator/scripts/make_excavation_slab_textured.py (seed {a.seed}, clutter {a.clutter}): rerun it instead of editing -->
<sdf version="1.6">
  <world name="Excavation_Slab_Textured">
    <scene>
      <ambient>0.42 0.43 0.46 1</ambient>
      <background>0.62 0.72 0.85 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <sky><time>14</time><sunrise>6</sunrise><sunset>20</sunset>
        <clouds><speed>0</speed><humidity>0.35</humidity><mean_size>0.6</mean_size></clouds></sky>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>{sun_dir[0]:.3f} {sun_dir[1]:.3f} {sun_dir[2]:.3f}</direction>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
    </light>
    <light name="sky_fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.22 0.24 0.28 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>{fill_dir[0]:.3f} {fill_dir[1]:.3f} {fill_dir[2]:.3f}</direction>
    </light>
{inc(lambda m: m == 'person')}
    <include><uri>model://{NAME}</uri><pose>0 0 0 0 0 0</pose></include>
{inc(lambda m: m != 'person')}
  </world>
</sdf>
'''
    out_dir = os.path.join(SIM, 'worlds', NAME); os.makedirs(out_dir, exist_ok=True)
    open(os.path.join(out_dir, f'{NAME}.world'), 'w').write(world)
    print(f'Wrote {MODEL} ({len(os.listdir(mesh_dir)) // 3} textured surfaces) and {out_dir}/{NAME}.world ({len(props)} props)')


if __name__ == '__main__':
    main()

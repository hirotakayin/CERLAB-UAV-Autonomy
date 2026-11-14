#!/usr/bin/env python3
"""Build the textured corridor: the corridor_dynamic_9 world (41 m x 5 m, 2.5 m walls, nine boxes, nine walking people)
with concrete walls and floor, plywood boxes, and random paintings on the walls, so a camera has texture to track
(optic flow, visual odometry). Everything is generated here, with a fixed seed:

  models/corridor_textured/          walls, boxes and paintings (static), its textures and OGRE material script
  worlds/corridor_textured/corridor_textured_dynamic_9.world   the people, light and ground, and the model above

  make_corridor_textured.py [--seed 7] [--paintings 16]

The walls keep one collision box each, as before; their look is 2.5 m square panels of a few concrete variants, so a
texture never stretches along the 41 m wall.
"""
import argparse
import copy
import os
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.normpath(os.path.join(HERE, '..'))
MODEL = os.path.join(SIM, 'models', 'corridor_textured')
X0, X1, Y0, Y1, H = -0.5, 40.5, -2.5, 2.5, 2.5  # inner faces of the walls (wall thickness 0.15, centred on these lines)
T = 0.15
BOXES = [((1.8, 1, 2), (9.597, -0.991)), ((1.8, 1, 2), (36.087, 0.922)), ((1.01, 1, 2), (5.192, 1.032)),
         ((1.01, 1, 2), (22.528, -1.287)), ((1.01, 1, 2), (3.322, -1.210)), ((1.01, 1, 2), (26.500, 0.751)),
         ((1.01, 1, 2), (16.652, 0.040)), ((1.01, 1, 2), (22.582, 1.860)), ((1.01, 1, 2), (30.283, 0.060))]


# ------------------------------------------------------------------ textures
def fractal(rng, size, octaves=6, persistence=0.55):
    out = np.zeros((size, size))
    amp, total = 1.0, 0.0
    for o in range(octaves):
        n = 2 ** (o + 2)
        layer = rng.random((n, n))
        layer = np.asarray(Image.fromarray((layer * 255).astype(np.uint8)).resize((size, size), Image.BICUBIC), float) / 255
        out += amp * layer
        total += amp
        amp *= persistence
    return out / total


def concrete(rng, size=1024, base=150, tie_holes=True):
    n = fractal(rng, size)
    img = base + (n - 0.5) * 70
    img += (fractal(rng, size, 3, 0.7) - 0.5) * 40  # large stains
    pores = rng.random((size, size)) < 0.004
    img[pores] -= rng.uniform(40, 90, pores.sum())
    img = np.clip(img, 0, 255)
    rgb = np.stack([img * 1.0, img * 0.98, img * 0.94], -1).clip(0, 255).astype(np.uint8)
    im = Image.fromarray(rgb).filter(ImageFilter.GaussianBlur(0.6))
    d = ImageDraw.Draw(im)
    if tie_holes:  # formwork tie holes and panel seams, like cast-in-place walls
        for gx in (0.2, 0.5, 0.8):
            for gy in (0.25, 0.75):
                x, y, r = gx * size, gy * size, size * 0.012
                d.ellipse([x - r, y - r, x + r, y + r], fill=(70, 68, 64))
        d.line([(0, size * 0.5), (size, size * 0.5)], fill=(115, 112, 106), width=3)
    for _ in range(rng.integers(1, 4)):  # a few hairline cracks
        x, y = rng.uniform(0, size, 2)
        pts = [(x, y)]
        for _ in range(rng.integers(6, 14)):
            x += rng.normal(0, size * 0.03); y += rng.normal(size * 0.03, size * 0.02)
            pts.append((x, y))
        d.line(pts, fill=(85, 82, 78), width=2)
    return im


def floor_concrete(rng, size=1024):
    im = concrete(rng, size, base=118, tie_holes=False)
    d = ImageDraw.Draw(im)
    for k in (0, size - 2):  # saw-cut joints at the panel edges
        d.line([(k, 0), (k, size)], fill=(60, 58, 55), width=4)
        d.line([(0, k), (size, k)], fill=(60, 58, 55), width=4)
    return im


def plywood(rng, size=512):
    y = np.linspace(0, 1, size)[:, None]
    grain = np.sin((y * 40 + fractal(rng, size, 4) * 6) * np.pi * 2) * 0.5 + 0.5
    img = 165 + grain * 35 + (fractal(rng, size) - 0.5) * 40
    rgb = np.stack([img, img * 0.78, img * 0.52], -1).clip(0, 255).astype(np.uint8)
    im = Image.fromarray(rgb)
    d = ImageDraw.Draw(im)
    d.rectangle([4, 4, size - 5, size - 5], outline=(90, 62, 35), width=10)
    d.text((size * 0.32, size * 0.45), 'FRAGILE', fill=(140, 30, 25))
    return im


PALETTES = [[(230, 57, 70), (241, 250, 238), (168, 218, 220), (69, 123, 157), (29, 53, 87)],
            [(38, 70, 83), (42, 157, 143), (233, 196, 106), (244, 162, 97), (231, 111, 81)],
            [(255, 190, 11), (251, 86, 7), (255, 0, 110), (131, 56, 236), (58, 134, 255)],
            [(13, 27, 42), (27, 38, 59), (65, 90, 119), (119, 141, 169), (224, 225, 221)],
            [(240, 240, 235), (220, 30, 30), (20, 60, 180), (250, 210, 30), (15, 15, 15)]]


def painting(rng, w=512, h=384):
    pal = PALETTES[rng.integers(len(PALETTES))]
    col = lambda: tuple(int(c) for c in pal[rng.integers(len(pal))])
    style = rng.integers(4)
    im = Image.new('RGB', (w, h), col())
    d = ImageDraw.Draw(im)
    if style == 0:  # grid of blocks, Mondrian-like
        xs = sorted(rng.uniform(0, w, rng.integers(2, 5))); ys = sorted(rng.uniform(0, h, rng.integers(2, 4)))
        xs, ys = [0] + list(xs) + [w], [0] + list(ys) + [h]
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                d.rectangle([xs[i], ys[j], xs[i + 1], ys[j + 1]], fill=col(), outline=(10, 10, 10), width=8)
    elif style == 1:  # circles and lines
        for _ in range(rng.integers(6, 14)):
            x, y, r = rng.uniform(0, w), rng.uniform(0, h), rng.uniform(15, 110)
            d.ellipse([x - r, y - r, x + r, y + r], fill=col(), outline=col(), width=int(rng.integers(2, 10)))
        for _ in range(rng.integers(3, 8)):
            d.line([tuple(rng.uniform(0, [w, h])), tuple(rng.uniform(0, [w, h]))], fill=col(), width=int(rng.integers(3, 12)))
    elif style == 2:  # landscape: sky, hills, sun
        top, bot = np.array(col(), float), np.array(col(), float)
        for y in range(h):
            t = y / h
            d.line([(0, y), (w, y)], fill=tuple((top * (1 - t) + bot * t).astype(int)))
        r = rng.uniform(25, 60); x = rng.uniform(r, w - r)
        d.ellipse([x - r, h * 0.2 - r, x + r, h * 0.2 + r], fill=col())
        for k in range(3):
            base = h * (0.5 + 0.15 * k)
            pts = [(0, h)] + [(x, base + 40 * np.sin(x / w * np.pi * rng.uniform(1, 4) + rng.uniform(0, 6))) for x in np.linspace(0, w, 30)] + [(w, h)]
            d.polygon(pts, fill=col())
    else:  # brush strokes
        for _ in range(rng.integers(40, 90)):
            x, y = rng.uniform(0, w), rng.uniform(0, h)
            pts = [(x, y)]
            for _ in range(rng.integers(3, 7)):
                x += rng.normal(0, 30); y += rng.normal(0, 20); pts.append((x, y))
            d.line(pts, fill=col(), width=int(rng.integers(6, 22)), joint='curve')
    d.rectangle([0, 0, w - 1, h - 1], outline=(92, 60, 30), width=18)  # wooden frame
    d.rectangle([18, 18, w - 19, h - 19], outline=(200, 170, 110), width=3)
    return im


# ------------------------------------------------------------------ SDF helpers
def material_xml(name):
    return ('<material><script><uri>model://corridor_textured/materials/scripts</uri>'
            '<uri>model://corridor_textured/materials/textures</uri><name>%s</name></script></material>' % name)


def box_visual(name, size, pose, mat):
    return ('<visual name="%s"><pose>%s</pose><geometry><box><size>%.3f %.3f %.3f</size></box></geometry>%s'
            '<cast_shadows>false</cast_shadows></visual>' % (name, ' '.join('%.4f' % v for v in pose), *size, material_xml(mat)))


def box_collision(name, size, pose):
    return ('<collision name="%s"><pose>%s</pose><geometry><box><size>%.3f %.3f %.3f</size></box></geometry></collision>'
            % (name, ' '.join('%.4f' % v for v in pose), *size))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--paintings', type=int, default=16)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    tex = os.path.join(MODEL, 'materials', 'textures'); scr = os.path.join(MODEL, 'materials', 'scripts')
    os.makedirs(tex, exist_ok=True); os.makedirs(scr, exist_ok=True)

    mats = []
    def add_mat(name, png, ambient=0.75, spec=0.05):
        mats.append('material %s\n{\n  technique\n  {\n    pass\n    {\n      ambient %.2f %.2f %.2f 1\n      diffuse 1 1 1 1\n'
                    '      specular %.2f %.2f %.2f 1 8\n      texture_unit\n      {\n        texture %s\n        filtering anisotropic\n'
                    '        max_anisotropy 8\n      }\n    }\n  }\n}\n' % (name, ambient, ambient, ambient, spec, spec, spec, png))
    nwall = 6
    for i in range(nwall):
        concrete(rng).save(os.path.join(tex, 'concrete_wall_%d.png' % i)); add_mat('Corridor/Wall%d' % i, 'concrete_wall_%d.png' % i)
    for i in range(3):
        floor_concrete(rng).save(os.path.join(tex, 'concrete_floor_%d.png' % i)); add_mat('Corridor/Floor%d' % i, 'concrete_floor_%d.png' % i, 0.7, 0.15)
    plywood(rng).save(os.path.join(tex, 'plywood.png')); add_mat('Corridor/Plywood', 'plywood.png')
    for i in range(a.paintings):
        painting(rng).save(os.path.join(tex, 'painting_%02d.png' % i)); add_mat('Corridor/Painting%02d' % i, 'painting_%02d.png' % i, 0.85, 0.2)
    open(os.path.join(scr, 'corridor_textured.material'), 'w').write('\n'.join(mats))

    parts = []
    # walls: one collision each, 2.5 m panels of concrete just inside it
    walls = [('north', (X1 - X0 + T, T, H), (20.0, Y1, H / 2), 'x'), ('south', (X1 - X0 + T, T, H), (20.0, Y0, H / 2), 'x'),
             ('west', (T, Y1 - Y0 + T, H), (X0, 0.0, H / 2), 'y'), ('east', (T, Y1 - Y0 + T, H), (X1, 0.0, H / 2), 'y')]
    for name, size, c, axis in walls:
        parts.append(box_collision('wall_%s' % name, size, (*c, 0, 0, 0)))
        lo, hi = (X0, X1) if axis == 'x' else (Y0, Y1)
        edges = np.arange(lo, hi + 1e-6, 2.5)
        if edges[-1] < hi - 1e-6: edges = np.append(edges, hi)
        for k in range(len(edges) - 1):
            mid, length = (edges[k] + edges[k + 1]) / 2, edges[k + 1] - edges[k]
            if axis == 'x':
                size_v, pos = (length, T, H), (mid, c[1], H / 2)
            else:
                size_v, pos = (T, length, H), (c[0], mid, H / 2)
            parts.append(box_visual('wall_%s_%d' % (name, k), size_v, (*pos, 0, 0, 0), 'Corridor/Wall%d' % rng.integers(nwall)))
    # floor: concrete slabs just above the ground plane (visual only; the ground plane collides)
    for i, x in enumerate(np.arange(X0, X1, 2.5)):
        for j, y in enumerate(np.arange(Y0, Y1, 2.5)):
            parts.append(box_visual('floor_%d_%d' % (i, j), (2.5, 2.5, 0.01), (x + 1.25, y + 1.25, 0.005, 0, 0, 0),
                                    'Corridor/Floor%d' % rng.integers(3)))
    # boxes as plywood crates
    for i, (size, (x, y)) in enumerate(BOXES):
        parts.append(box_collision('box_%d' % i, size, (x, y, size[2] / 2, 0, 0, 0)))
        parts.append(box_visual('box_%d_visual' % i, size, (x, y, size[2] / 2, 0, 0, 0), 'Corridor/Plywood'))
    # paintings: thin frames on the inner faces of the long walls, kept clear of each other
    placed = {Y0: [], Y1: []}
    k = 0
    while k < a.paintings:
        wall = Y0 if rng.random() < 0.5 else Y1
        w, h = rng.uniform(0.9, 1.6), rng.uniform(0.6, 1.1)
        x = rng.uniform(X0 + 1.0, X1 - 1.0)
        if any(abs(x - px) < (w + pw) / 2 + 0.4 for px, pw in placed[wall]): continue
        placed[wall].append((x, w))
        zc = rng.uniform(1.0, 1.75)
        y = wall + (T / 2 + 0.012) * (1 if wall == Y0 else -1)
        yaw = np.pi / 2 if wall == Y0 else -np.pi / 2  # box x axis along the wall's thickness: its +x face looks into the corridor
        parts.append(box_visual('painting_%02d' % k, (0.02, w, h), (x, y, zc, 0, 0, yaw), 'Corridor/Painting%02d' % k))
        k += 1

    sdf = ('<?xml version="1.0"?>\n<!-- generated by uav_simulator/scripts/make_corridor_textured.py: rerun it instead of editing -->\n'
           '<sdf version="1.6"><model name="corridor_textured"><static>true</static><link name="link">\n%s\n</link></model></sdf>\n'
           % '\n'.join(parts))
    open(os.path.join(MODEL, 'corridor_textured.sdf'), 'w').write(sdf)
    open(os.path.join(MODEL, 'model.config'), 'w').write(
        '<?xml version="1.0"?>\n<model><name>corridor_textured</name><version>1.0</version><sdf version="1.6">corridor_textured.sdf</sdf>\n'
        '<description>corridor_dynamic_9 with concrete walls and floor, plywood boxes and %d random paintings (seed %d). '
        'Inner faces: x [%.1f, %.1f], y [%.1f, %.1f], walls %.1f m tall, open top.</description></model>\n'
        % (a.paintings, a.seed, X0, X1, Y0, Y1, H))

    # world: the dynamic corridor's people, light, physics and ground, with this model for the corridor and boxes
    tree = ET.parse(os.path.join(SIM, 'worlds', 'corridor', 'corridor_dynamic_9.world'))
    world = tree.getroot().find('world')
    for child in list(world):
        name = child.get('name') or ''
        if child.tag == 'state' or (child.tag == 'model' and (name == 'corridor' or name.startswith('box_'))):
            world.remove(child)
    # the people: give each a body you can see and hit (their only collision was a 2 cm plate at their feet, so ray
    # sensors and the drone went straight through them). Size as in their names: 0.5 x 0.5 x 1.8 m
    for person in [x for x in world.findall('model') if (x.get('name') or '').startswith('person')]:
        link = person.find('link')
        if link.find("collision[@name='body']") is None:
            col = ET.SubElement(link, 'collision', name='body')
            ET.SubElement(col, 'pose').text = '0 0 0.9 0 0 0'
            ET.SubElement(ET.SubElement(ET.SubElement(col, 'geometry'), 'box'), 'size').text = '0.5 0.5 1.8'
    inc = ET.SubElement(world, 'include'); ET.SubElement(inc, 'uri').text = 'model://corridor_textured'
    ET.SubElement(inc, 'pose').text = '0 0 0 0 0 0'
    out_dir = os.path.join(SIM, 'worlds', 'corridor_textured'); os.makedirs(out_dir, exist_ok=True)
    tree.write(os.path.join(out_dir, 'corridor_textured_dynamic_9.world'), xml_declaration=True)
    # and the same without the people, to tell sensing problems from moving-obstacle problems
    for child in list(world):
        if child.tag == 'model' and (child.get('name') or '').startswith('person'): world.remove(child)
    tree.write(os.path.join(out_dir, 'corridor_textured_static.world'), xml_declaration=True)
    print('Wrote', MODEL, 'and corridor_textured_dynamic_9.world, corridor_textured_static.world in', out_dir)


if __name__ == '__main__':
    main()

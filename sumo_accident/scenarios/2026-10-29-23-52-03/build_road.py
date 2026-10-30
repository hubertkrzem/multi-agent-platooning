"""Build road.net.xml from the OSM Web Wizard export (osm.net.xml.gz).

- Keeps only the two motorway carriageways (no ramps) and makes every edge two lanes.
- Shortens both carriageways so they start and end level with each other, keeping the
  middle KEEP_FRACTION of the stretch where they run side by side.
- Keeps direction A on its real geometry and redraws direction B as a parallel copy of it
  at a constant distance (the average measured median), so the two line up everywhere.
- Lets netconvert recompute all lane connections.

Run from this folder:  python build_road.py
"""
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.append(os.path.join(os.environ.get(
    "SUMO_HOME", "/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo"),
    "tools"))
from sumolib.geomhelper import (distance, distancePointToPolygon, move2side, polyLength,  # noqa: E402
                                polygonOffsetWithMinimumDistanceToPoint, positionAtShapeOffset)

SOURCE_NET = "osm.net.xml.gz"
OUTPUT_NET = "road.net.xml"
NUM_LANES = 2
LANE_WIDTH = 3.2  # SUMO default, used where the export gives no width
KEEP_FRACTION = 0.5  # share of the stretch where both carriageways run side by side

# Main carriageway edges, in driving order
DIRECTION_A = ["4402438", "70869582", "1229067689#1-AddedOnRampEdge", "1229067689#1",
               "783429011", "1229067686"]
DIRECTION_B = ["1068016336", "783429024#0", "783429024#1", "783429025", "783429026",
               "783429027", "23024719", "70869586"]


def sub_shape(shape, start, end):
    """Part of a polyline between two offsets (start < end), measured along it."""
    result = [positionAtShapeOffset(shape, start)]
    travelled = 0
    for p1, p2 in zip(shape, shape[1:]):
        travelled += distance(p1, p2)
        if start < travelled < end:
            result.append(p2)
    result.append(positionAtShapeOffset(shape, end))
    return result


def parallel_line(shape, amount, step=5, smoothing=20):
    """A line at a constant sideways distance from shape (positive = left).

    The direction at each point is taken over +-smoothing metres, so the line stays
    smooth on the inside of bends instead of folding back on itself.
    """
    length = polyLength(shape)
    result = []
    for offset in [d for d in range(0, int(length), step)] + [length]:
        x1, y1 = positionAtShapeOffset(shape, max(offset - smoothing, 0))
        x2, y2 = positionAtShapeOffset(shape, min(offset + smoothing, length))
        dx, dy = x2 - x1, y2 - y1
        norm = (dx * dx + dy * dy) ** 0.5
        x, y = positionAtShapeOffset(shape, offset)
        result.append((x - dy / norm * amount, y + dx / norm * amount))
    return result


def fmt(shape):
    return " ".join("%.2f,%.2f" % (x, y) for x, y in shape)


with tempfile.TemporaryDirectory() as tmp:
    prefix = os.path.join(tmp, "plain")

    # 1. Export just the kept edges as plain XML
    subprocess.run(["netconvert", "-s", SOURCE_NET,
                    "--keep-edges.explicit", ",".join(DIRECTION_A + DIRECTION_B),
                    "--plain-output-prefix", prefix], check=True)

    nod_file, edg_file = prefix + ".nod.xml", prefix + ".edg.xml"
    nodes_tree, edges_tree = ET.parse(nod_file), ET.parse(edg_file)
    nodes = {n.get("id"): n for n in nodes_tree.getroot().iter("node")}
    edges = {e.get("id"): e for e in edges_tree.getroot().iter("edge")}

    def node_pos(node_id):
        return float(nodes[node_id].get("x")), float(nodes[node_id].get("y"))

    def edge_shape(edge):
        if edge.get("shape"):
            return [tuple(map(float, p.split(","))) for p in edge.get("shape").split()]
        return [node_pos(edge.get("from")), node_pos(edge.get("to"))]

    def chain(edge_ids):
        points = []
        for edge_id in edge_ids:
            shape = edge_shape(edges[edge_id])
            points += shape[1:] if points and distance(points[-1], shape[0]) < 0.1 else shape
        return points

    # 2. Reduce the merge/diverge sections (3-4 lanes where ramps joined) to two lanes.
    #    Their centre line sat in the middle of the wider road, so move it sideways to
    #    line up with the neighbouring edges; otherwise the lanes step at each end.
    for edge_ids in (DIRECTION_A, DIRECTION_B):
        for i, edge_id in enumerate(edge_ids):
            edge = edges[edge_id]
            removed = int(edge.get("numLanes")) - NUM_LANES
            edge.set("numLanes", str(NUM_LANES))
            for lane in list(edge.findall("lane")):
                if int(lane.get("index")) >= NUM_LANES:
                    edge.remove(lane)
            if removed <= 0:
                continue
            shape = edge_shape(edge)
            before = edge_shape(edges[edge_ids[i - 1]])[-1] if i > 0 else None
            after = edge_shape(edges[edge_ids[i + 1]])[0] if i + 1 < len(edge_ids) else None

            def misfit(candidate):
                return ((distance(candidate[0], before) if before else 0) +
                        (distance(candidate[-1], after) if after else 0))

            amount = removed * float(edge.get("width", LANE_WIDTH)) / 2
            edge.set("shape", fmt(min(move2side(shape, amount), move2side(shape, -amount), key=misfit)))

    # 3. Cut direction A to the part alongside direction B (A runs on past B's start),
    #    then keep only the middle KEEP_FRACTION of that, cutting both ends equally
    shape_a = chain(DIRECTION_A)
    overlap = polygonOffsetWithMinimumDistanceToPoint(edge_shape(edges[DIRECTION_B[0]])[0], shape_a)
    keep_start = overlap * (1 - KEEP_FRACTION) / 2
    keep_end = keep_start + overlap * KEEP_FRACTION

    def add_node(node_id, point):
        nodes[node_id] = ET.SubElement(nodes_tree.getroot(), "node", id=node_id, type="dead_end",
                                       x="%.2f" % point[0], y="%.2f" % point[1])

    travelled = 0
    for edge_id in DIRECTION_A:
        edge = edges[edge_id]
        shape = edge_shape(edge)
        length = polyLength(shape)
        if travelled + length <= keep_start or travelled >= keep_end:
            edges_tree.getroot().remove(edge)
        else:
            new_shape = sub_shape(shape, max(keep_start - travelled, 0), min(keep_end - travelled, length))
            if travelled < keep_start:
                add_node("A_start", new_shape[0])
                edge.set("from", "A_start")
            if travelled + length > keep_end:
                add_node("A_end", new_shape[-1])
                edge.set("to", "A_end")
            edge.set("shape", fmt(new_shape))
        travelled += length
    shape_a = sub_shape(shape_a, keep_start, keep_end)
    cut_a = keep_end - keep_start

    # 4. Redraw direction B parallel to A. B's nodes are moved onto the parallel line; any
    #    node beyond the ends of A is pulled back to the end, which shortens B to match A.
    shape_b = chain(DIRECTION_B)
    mid_b = positionAtShapeOffset(shape_b, polyLength(shape_b) / 2)
    gap = sum(distancePointToPolygon(p, shape_b) for p in
              (positionAtShapeOffset(shape_a, d) for d in range(0, int(cut_a), 50))) / (int(cut_a) // 50 + 1)
    parallel = min((parallel_line(shape_a, gap), parallel_line(shape_a, -gap)),
                   key=lambda s: distancePointToPolygon(mid_b, s))
    parallel_length = polyLength(parallel)

    def parallel_offset(point):
        return min(max(polygonOffsetWithMinimumDistanceToPoint(point, parallel), 0), parallel_length)

    node_offsets = {}
    for edge_id in DIRECTION_B:
        for node_id in (edges[edge_id].get("from"), edges[edge_id].get("to")):
            node_offsets[node_id] = parallel_offset(node_pos(node_id))
    for node_id, offset in node_offsets.items():
        x, y = positionAtShapeOffset(parallel, offset)
        nodes[node_id].set("x", "%.2f" % x)
        nodes[node_id].set("y", "%.2f" % y)
    for edge_id in DIRECTION_B:
        edge = edges[edge_id]
        start, end = node_offsets[edge.get("to")], node_offsets[edge.get("from")]
        if end - start < 1:  # entirely beyond the ends of A
            edges_tree.getroot().remove(edge)
        else:  # B runs opposite to A, so walk the parallel line backwards
            edge.set("shape", fmt(list(reversed(sub_shape(parallel, start, end)))))

    # 5. Let netconvert recompute junction shapes (they were sized for the ramps)
    for node in nodes.values():
        node.attrib.pop("shape", None)
    nodes_tree.write(nod_file)
    edges_tree.write(edg_file)

    # 6. Rebuild without the old connection file, so all connections are recomputed.
    #    Brisbane drives on the left, which the plain files do not carry over.
    subprocess.run(["netconvert", "-n", nod_file, "-e", edg_file,
                    "-t", prefix + ".typ.xml", "--lefthand",
                    "-o", OUTPUT_NET], check=True)

    kept = [e.get("id") for e in edges_tree.getroot().iter("edge")]

print("Wrote %s (median gap %.1f m, length %.0f m)" % (OUTPUT_NET, gap, cut_a))
print("Direction A edges:", [e for e in DIRECTION_A if e in kept])
print("Direction B edges:", [e for e in DIRECTION_B if e in kept])

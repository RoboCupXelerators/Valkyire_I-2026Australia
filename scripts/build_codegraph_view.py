from __future__ import annotations

import ast
import builtins
import html
import json
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT / "src"
OUT_PATH = ROOT / "docs" / "codegraph_view.html"
BUILTIN_NAMES = set(dir(builtins))

EXCLUDE_PARTS = {
    "__pycache__",
    ".pytest_cache",
    "tests",
}

DATA_SYMBOLS = {
    "GetBallPos",
    "get_ball_pos",
    "get_vision_data",
    "RobotBallPos",
    "FieldBallPos",
    "FieldRobotPose",
    "GetPose",
    "HasBall",
    "SetWheelSpeed",
    "SetMotor",
    "RobotMoveVector",
    "FieldMoveVector",
    "Pos2Pos",
    "Navigate2",
    "LockBallSlip",
}

FACADE_SYMBOL_TARGETS = {
    "chassis": "src/LunaPre/motion/chassis.py",
    "peripheral": "src/LunaPre/motion/chassis.py",
    "key": "src/LunaPre/motion/chassis.py",
    "UnitedPosition": "src/LunaPre/positions.py",
    "Positions": "src/LunaPre/positions.py",
    "RecordingPositions": "src/LunaPre/position_recorder.py",
    "Settings": "src/ReasonData/config.py",
    "logger": "src/ReasonData/__init__.py",
}


@dataclass
class FileInfo:
    path: str
    module: str
    group: str
    lines: int
    functions: list[str] = field(default_factory=list)
    classes: list[str] = field(default_factory=list)


@dataclass
class RawEdge:
    source: str
    target: str
    kind: str
    label: str
    line: int


def is_source_file(path: Path) -> bool:
    if path.suffix != ".py":
        return False
    rel_parts = path.relative_to(ROOT).parts
    return not any(part in EXCLUDE_PARTS for part in rel_parts)


def module_name(path: Path) -> str:
    rel = path.relative_to(ROOT).with_suffix("")
    return ".".join(rel.parts)


def group_for(path: str) -> str:
    if path.startswith("src/LunaPre/"):
        return "LunaPre"
    if path.startswith("src/vision_utils/"):
        return "vision"
    if path.startswith("src/utils/"):
        return "bits/utils"
    if path.startswith("src/mlp/"):
        return "mlp"
    if path.startswith("src/teacher/"):
        return "teacher"
    if path.startswith("src/ReasonData/"):
        return "config/data"
    if path in {"src/offence.py", "src/defence.py", "src/alpha.py", "src/lambda_strategy.py", "src/BasicFuc.py"}:
        return "strategy"
    if path == "src/control_panel.py":
        return "control"
    return "other"


def local_module_to_path(module: str, module_to_file: dict[str, str]) -> str | None:
    candidates = [module, f"{module}.__init__"]
    if not module.startswith("src."):
        candidates.extend([f"src.{module}", f"src.{module}.__init__"])
    for candidate in candidates:
        if candidate in module_to_file:
            return module_to_file[candidate]
    return None


def call_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        parts = [node.attr]
        value = node.value
        while isinstance(value, ast.Attribute):
            parts.append(value.attr)
            value = value.value
        if isinstance(value, ast.Name):
            parts.append(value.id)
        return ".".join(reversed(parts))
    return None


def dynamic_import_target(node: ast.Call, rel: str, current_module: str, module_to_file: dict[str, str]) -> str | None:
    name = call_name(node.func)
    if name not in {"importlib.import_module", "import_module"}:
        return None
    if not node.args or not isinstance(node.args[0], ast.Constant) or not isinstance(node.args[0].value, str):
        return None

    module = node.args[0].value
    if module.startswith("."):
        package = current_module.rsplit(".", 1)[0]
        module = package + module
    return local_module_to_path(module, module_to_file)


class FileVisitor(ast.NodeVisitor):
    def __init__(
        self,
        rel_path: str,
        imports: dict[str, str],
        name_to_files: dict[str, set[str]],
        attr_to_files: dict[str, set[str]],
    ):
        self.rel_path = rel_path
        self.imports = imports
        self.name_to_files = name_to_files
        self.attr_to_files = attr_to_files
        self.edges: list[RawEdge] = []

    def visit_Call(self, node: ast.Call) -> None:
        name = call_name(node.func)
        if name:
            target = self.resolve_call_target(name)
            if target and target != self.rel_path:
                symbol = name.split(".")[-1]
                kind = "data" if symbol in DATA_SYMBOLS else "call"
                self.edges.append(RawEdge(self.rel_path, target, kind, name, node.lineno))
        self.generic_visit(node)

    def resolve_call_target(self, name: str) -> str | None:
        parts = name.split(".")
        if len(parts) >= 2:
            root = parts[0]
            if root in {"self", "cls", "super"}:
                return None
            if root in self.imports:
                return self.imports[root]
        else:
            if name in BUILTIN_NAMES:
                return None
            files = self.name_to_files.get(name, set())
            if len(files) == 1:
                return next(iter(files))
        return None


def parse_files() -> tuple[dict[str, FileInfo], list[RawEdge], dict[str, list[str]]]:
    paths = sorted(path for path in SRC_ROOT.rglob("*.py") if is_source_file(path))
    module_to_file = {module_name(path): str(path.relative_to(ROOT)) for path in paths}
    file_infos: dict[str, FileInfo] = {}
    name_to_files: dict[str, set[str]] = defaultdict(set)
    attr_to_files: dict[str, set[str]] = defaultdict(set)
    trees: dict[str, ast.Module] = {}

    for path in paths:
        rel = str(path.relative_to(ROOT))
        text = path.read_text(errors="ignore")
        try:
            tree = ast.parse(text)
        except SyntaxError:
            continue
        trees[rel] = tree
        info = FileInfo(rel, module_name(path), group_for(rel), len(text.splitlines()))

        class DefVisitor(ast.NodeVisitor):
            def __init__(self):
                self.class_stack: list[str] = []

            def visit_ClassDef(self, node: ast.ClassDef) -> None:
                info.classes.append(node.name)
                name_to_files[node.name].add(rel)
                attr_to_files[node.name].add(rel)
                self.class_stack.append(node.name)
                self.generic_visit(node)
                self.class_stack.pop()

            def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
                qname = ".".join(self.class_stack + [node.name]) if self.class_stack else node.name
                info.functions.append(qname)
                name_to_files[node.name].add(rel)
                attr_to_files[node.name].add(rel)
                self.generic_visit(node)

            visit_AsyncFunctionDef = visit_FunctionDef

        DefVisitor().visit(tree)
        file_infos[rel] = info

    raw_edges: list[RawEdge] = []
    unresolved_imports: dict[str, list[str]] = defaultdict(list)

    for rel, tree in trees.items():
        imports: dict[str, str] = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    target = local_module_to_path(alias.name, module_to_file)
                    if target:
                        local_name = alias.asname or alias.name.split(".")[0]
                        imports[local_name] = target
                        raw_edges.append(RawEdge(rel, target, "import", alias.name, node.lineno))
            elif isinstance(node, ast.ImportFrom):
                module = node.module or ""
                if node.level:
                    current = file_infos[rel].module.split(".")
                    base = current[:-node.level]
                    module = ".".join(base + ([module] if module else []))
                elif not module:
                    continue
                module_target = local_module_to_path(module, module_to_file)
                for alias in node.names:
                    if alias.name == "*" and module in {"LunaPre", "src.LunaPre"}:
                        for symbol, target in FACADE_SYMBOL_TARGETS.items():
                            imports.setdefault(symbol, target)
                            raw_edges.append(RawEdge(rel, target, "import", f"{module}.{symbol}", node.lineno))
                        continue
                    symbol_module = f"{module}.{alias.name}"
                    target = local_module_to_path(symbol_module, module_to_file)
                    if target is None and module in {"LunaPre", "src.LunaPre"}:
                        target = FACADE_SYMBOL_TARGETS.get(alias.name)
                    if target is None:
                        target = module_target
                    if target:
                        imports[alias.asname or alias.name] = target
                        raw_edges.append(RawEdge(rel, target, "import", f"{module}.{alias.name}", node.lineno))
                    elif module.startswith(("src", "LunaPre", "utils", "vision_utils", "ReasonData")):
                        unresolved_imports[rel].append(f"{module}.{alias.name}")
            elif isinstance(node, ast.Call):
                target = dynamic_import_target(node, rel, file_infos[rel].module, module_to_file)
                if target:
                    raw_edges.append(RawEdge(rel, target, "import", call_name(node.func) or "import_module", node.lineno))

        visitor = FileVisitor(rel, imports, name_to_files, attr_to_files)
        visitor.visit(tree)
        raw_edges.extend(visitor.edges)

    return file_infos, raw_edges, unresolved_imports


def build_payload() -> dict:
    file_infos, raw_edges, unresolved_imports = parse_files()
    edge_map: dict[tuple[str, str, str], dict] = {}
    for edge in raw_edges:
        if edge.source not in file_infos or edge.target not in file_infos:
            continue
        key = (edge.source, edge.target, edge.kind)
        item = edge_map.setdefault(
            key,
            {"source": edge.source, "target": edge.target, "kind": edge.kind, "weight": 0, "labels": []},
        )
        item["weight"] += 1
        if len(item["labels"]) < 8:
            item["labels"].append(f"{edge.label}:{edge.line}")

    incoming = Counter()
    outgoing = Counter()
    for edge in edge_map.values():
        incoming[edge["target"]] += edge["weight"]
        outgoing[edge["source"]] += edge["weight"]

    nodes = []
    for rel, info in file_infos.items():
        nodes.append(
            {
                "id": rel,
                "label": Path(rel).name,
                "path": rel,
                "module": info.module,
                "group": info.group,
                "lines": info.lines,
                "function_count": len(info.functions),
                "class_count": len(info.classes),
                "functions": info.functions[:40],
                "classes": info.classes[:40],
                "incoming": incoming[rel],
                "outgoing": outgoing[rel],
                "consumed_by": incoming[rel],
                "consumes": outgoing[rel],
            }
        )

    edges = sorted(edge_map.values(), key=lambda e: (e["source"], e["target"], e["kind"]))
    hotspots = sorted(nodes, key=lambda n: (n["incoming"] + n["outgoing"], n["function_count"]), reverse=True)[:20]
    duplicate_names = []
    by_function = defaultdict(list)
    for info in file_infos.values():
        for fn in info.functions:
            by_function[fn.split(".")[-1]].append(info.path)
    for name, paths in by_function.items():
        unique = sorted(set(paths))
        if len(unique) >= 3 and name not in {"__init__", "main", "close", "to_dict", "from_dict"}:
            duplicate_names.append({"name": name, "files": unique[:12], "count": len(unique)})
    duplicate_names.sort(key=lambda item: (-item["count"], item["name"]))

    return {
        "generated_from": "Software/src",
        "node_count": len(nodes),
        "edge_count": len(edges),
        "nodes": nodes,
        "edges": edges,
        "hotspots": hotspots,
        "duplicate_names": duplicate_names[:30],
        "unresolved_imports": dict(sorted(unresolved_imports.items())),
    }


HTML_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Software Codegraph</title>
<style>
:root {
  --bg: #f7f7f3;
  --panel: #ffffff;
  --ink: #252525;
  --muted: #6f6f67;
  --line: #d9d7cf;
  --accent: #1f7a8c;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--ink);
  font: 14px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
header {
  height: 52px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 18px;
  border-bottom: 1px solid var(--line);
  background: #fbfbf8;
}
h1 { font-size: 17px; margin: 0; font-weight: 650; letter-spacing: 0; }
.meta { color: var(--muted); font-size: 12px; }
.layout {
  display: grid;
  grid-template-columns: 330px minmax(520px, 1fr) 360px;
  height: calc(100vh - 52px);
}
aside, .details {
  overflow: auto;
  background: var(--panel);
  border-right: 1px solid var(--line);
}
.details { border-right: 0; border-left: 1px solid var(--line); }
.section { padding: 14px; border-bottom: 1px solid var(--line); }
.section h2 { font-size: 13px; margin: 0 0 10px; color: #333; }
input, select {
  width: 100%;
  height: 32px;
  border: 1px solid var(--line);
  border-radius: 6px;
  padding: 0 9px;
  background: #fff;
}
label { display: flex; gap: 8px; align-items: center; margin: 8px 0; color: #333; }
label input { width: auto; height: auto; }
.chip {
  display: inline-block;
  padding: 2px 7px;
  border-radius: 999px;
  background: #eceae2;
  color: #4d4a42;
  font-size: 12px;
  margin: 2px 4px 2px 0;
}
.list { display: grid; gap: 6px; }
.item {
  border: 1px solid var(--line);
  border-radius: 7px;
  padding: 8px;
  cursor: pointer;
  background: #fff;
}
.item:hover { border-color: var(--accent); }
.item strong { display: block; font-size: 13px; }
.item small { color: var(--muted); word-break: break-all; }
main { position: relative; overflow: hidden; }
svg { width: 100%; height: 100%; display: block; background: #f4f3ee; }
.edge { stroke-opacity: .28; fill: none; }
.edge.import { stroke: #6b7280; }
.edge.call { stroke: #8a6d3b; }
.edge.data { stroke: #c2410c; stroke-opacity: .55; }
.node circle { stroke: #fff; stroke-width: 1.5; }
.node text { pointer-events: none; font-size: 10px; fill: #232323; paint-order: stroke; stroke: #f4f3ee; stroke-width: 3px; }
.selected circle { stroke: #111; stroke-width: 3; }
.hidden { display: none; }
.toolbar {
  position: absolute;
  left: 12px;
  top: 12px;
  padding: 8px 10px;
  border: 1px solid var(--line);
  background: rgba(255,255,255,.92);
  border-radius: 8px;
  color: var(--muted);
  font-size: 12px;
}
pre {
  white-space: pre-wrap;
  word-break: break-word;
  background: #f4f3ee;
  border: 1px solid var(--line);
  border-radius: 7px;
  padding: 9px;
  max-height: 240px;
  overflow: auto;
}
.empty { color: var(--muted); }
@media (max-width: 1100px) {
  .layout { grid-template-columns: 280px 1fr; }
  .details { display: none; }
}
</style>
</head>
<body>
<header>
  <h1>Software Codegraph</h1>
  <div class="meta" id="summary"></div>
</header>
<div class="layout">
  <aside>
    <div class="section">
      <h2>过滤</h2>
      <input id="search" placeholder="搜索文件 / 函数 / 分组">
      <label><input id="showImport" type="checkbox" checked> import 边</label>
      <label><input id="showCall" type="checkbox" checked> call 边</label>
      <label><input id="showData" type="checkbox" checked> data 边</label>
      <select id="groupFilter">
        <option value="">所有分组</option>
      </select>
    </div>
    <div class="section">
      <h2>热点文件</h2>
      <div class="list" id="hotspots"></div>
    </div>
    <div class="section">
      <h2>重复函数名</h2>
      <div class="list" id="duplicates"></div>
    </div>
  </aside>
  <main>
    <div class="toolbar">拖动节点；滚轮缩放；点击节点看上下游。</div>
    <svg id="graph"></svg>
  </main>
  <div class="details">
    <div class="section">
      <h2>选中节点</h2>
      <div id="selected" class="empty">点击一个文件节点。</div>
    </div>
    <div class="section">
      <h2>上游：它消费/依赖谁</h2>
      <div id="incoming"></div>
    </div>
    <div class="section">
      <h2>下游：谁消费/调用它</h2>
      <div id="outgoing"></div>
    </div>
  </div>
</div>
<script>
const DATA = __DATA__;
const GROUP_COLORS = {
  "strategy": "#e76f51",
  "LunaPre": "#2a9d8f",
  "vision": "#457b9d",
  "bits/utils": "#6d597a",
  "mlp": "#8ab17d",
  "teacher": "#b56576",
  "config/data": "#d4a373",
  "control": "#577590",
  "other": "#7b7b74"
};

const nodes = DATA.nodes.map((n, i) => ({...n, index: i, x: 160 + (i % 12) * 70, y: 120 + Math.floor(i / 12) * 55, vx: 0, vy: 0}));
const nodeById = new Map(nodes.map(n => [n.id, n]));
const edges = DATA.edges.map(e => ({...e, sourceNode: nodeById.get(e.source), targetNode: nodeById.get(e.target)})).filter(e => e.sourceNode && e.targetNode);

const svg = document.getElementById("graph");
const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
const edgeLayer = document.createElementNS("http://www.w3.org/2000/svg", "g");
const nodeLayer = document.createElementNS("http://www.w3.org/2000/svg", "g");
g.append(edgeLayer, nodeLayer);
svg.append(g);

let scale = 1, panX = 0, panY = 0, selected = null;
let dragging = null;

document.getElementById("summary").textContent = `${DATA.node_count} files / ${DATA.edge_count} edges`;
const groups = [...new Set(nodes.map(n => n.group))].sort();
const groupFilter = document.getElementById("groupFilter");
groups.forEach(group => {
  const option = document.createElement("option");
  option.value = group;
  option.textContent = group;
  groupFilter.append(option);
});

function makeItem(title, subtitle, onClick) {
  const item = document.createElement("div");
  item.className = "item";
  item.innerHTML = `<strong>${escapeHtml(title)}</strong><small>${escapeHtml(subtitle)}</small>`;
  item.onclick = onClick;
  return item;
}

document.getElementById("hotspots").replaceChildren(...DATA.hotspots.slice(0, 14).map(n =>
  makeItem(n.label, `${n.group} · consumes ${n.consumes} / consumed by ${n.consumed_by} · ${n.path}`, () => selectNode(nodeById.get(n.id)))
));
document.getElementById("duplicates").replaceChildren(...DATA.duplicate_names.slice(0, 12).map(d =>
  makeItem(d.name, `${d.count} files: ${d.files.slice(0, 3).join(", ")}`, () => {
    document.getElementById("search").value = d.name;
    render();
  })
));

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}

function passesNodeFilter(node) {
  const q = document.getElementById("search").value.trim().toLowerCase();
  const group = groupFilter.value;
  if (group && node.group !== group) return false;
  if (!q) return true;
  return node.path.toLowerCase().includes(q)
    || node.group.toLowerCase().includes(q)
    || node.functions.some(fn => fn.toLowerCase().includes(q))
    || node.classes.some(cls => cls.toLowerCase().includes(q));
}

function passesEdgeFilter(edge) {
  if (edge.kind === "import" && !document.getElementById("showImport").checked) return false;
  if (edge.kind === "call" && !document.getElementById("showCall").checked) return false;
  if (edge.kind === "data" && !document.getElementById("showData").checked) return false;
  return passesNodeFilter(edge.sourceNode) && passesNodeFilter(edge.targetNode);
}

function render() {
  edgeLayer.replaceChildren();
  nodeLayer.replaceChildren();
  const visibleEdges = edges.filter(passesEdgeFilter);
  const connected = new Set();
  visibleEdges.forEach(e => { connected.add(e.source); connected.add(e.target); });
  const visibleNodes = nodes.filter(n => passesNodeFilter(n) && (connected.has(n.id) || document.getElementById("search").value));

  visibleEdges.forEach(edge => {
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.classList.add("edge", edge.kind);
    line.setAttribute("stroke-width", String(Math.min(5, 0.7 + Math.sqrt(edge.weight) * 0.55)));
    line.dataset.source = edge.source;
    line.dataset.target = edge.target;
    edgeLayer.append(line);
  });

  visibleNodes.forEach(node => {
    const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
    group.classList.add("node");
    if (selected && selected.id === node.id) group.classList.add("selected");
    group.dataset.id = node.id;
    const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    circle.setAttribute("r", String(5 + Math.min(14, Math.sqrt(node.function_count + node.class_count) * 2.2)));
    circle.setAttribute("fill", GROUP_COLORS[node.group] || GROUP_COLORS.other);
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    text.textContent = node.label;
    text.setAttribute("x", "9");
    text.setAttribute("y", "3");
    group.append(circle, text);
    group.addEventListener("mousedown", ev => { dragging = node; ev.preventDefault(); });
    group.addEventListener("click", ev => { selectNode(node); ev.stopPropagation(); });
    nodeLayer.append(group);
  });
  tick(true);
}

function tick(skipPhysics = false) {
  if (!skipPhysics) {
    const visibleNodes = nodes.filter(passesNodeFilter);
    const visibleEdges = edges.filter(passesEdgeFilter);
    for (let i = 0; i < visibleNodes.length; i++) {
      for (let j = i + 1; j < visibleNodes.length; j++) {
        const a = visibleNodes[i], b = visibleNodes[j];
        let dx = a.x - b.x, dy = a.y - b.y;
        let d2 = dx * dx + dy * dy + 0.01;
        const f = Math.min(1600 / d2, 2.5);
        const d = Math.sqrt(d2);
        a.vx += (dx / d) * f; a.vy += (dy / d) * f;
        b.vx -= (dx / d) * f; b.vy -= (dy / d) * f;
      }
    }
    visibleEdges.forEach(e => {
      const a = e.sourceNode, b = e.targetNode;
      const dx = b.x - a.x, dy = b.y - a.y;
      const d = Math.sqrt(dx * dx + dy * dy) || 1;
      const target = 120;
      const f = (d - target) * 0.006;
      a.vx += dx / d * f; a.vy += dy / d * f;
      b.vx -= dx / d * f; b.vy -= dy / d * f;
    });
    const rect = svg.getBoundingClientRect();
    visibleNodes.forEach(n => {
      n.vx += (rect.width / 2 - n.x) * 0.0008;
      n.vy += (rect.height / 2 - n.y) * 0.0008;
      n.x += n.vx; n.y += n.vy;
      n.vx *= 0.82; n.vy *= 0.82;
    });
  }

  edgeLayer.querySelectorAll("line").forEach(line => {
    const a = nodeById.get(line.dataset.source), b = nodeById.get(line.dataset.target);
    line.setAttribute("x1", a.x); line.setAttribute("y1", a.y);
    line.setAttribute("x2", b.x); line.setAttribute("y2", b.y);
  });
  nodeLayer.querySelectorAll(".node").forEach(group => {
    const n = nodeById.get(group.dataset.id);
    group.setAttribute("transform", `translate(${n.x},${n.y})`);
  });
  g.setAttribute("transform", `translate(${panX},${panY}) scale(${scale})`);
}

function selectNode(node) {
  selected = node;
  render();
  const incoming = edges.filter(e => e.target === node.id).sort((a, b) => b.weight - a.weight);
  const outgoing = edges.filter(e => e.source === node.id).sort((a, b) => b.weight - a.weight);
  document.getElementById("selected").innerHTML = `
    <strong>${escapeHtml(node.path)}</strong>
    <div><span class="chip">${escapeHtml(node.group)}</span><span class="chip">${node.lines} lines</span><span class="chip">${node.function_count} funcs</span></div>
    <pre>${escapeHtml(node.functions.slice(0, 30).join("\\n") || "(no functions)")}</pre>`;
  document.getElementById("incoming").replaceChildren(...edgeItems(outgoing, "target"));
  document.getElementById("outgoing").replaceChildren(...edgeItems(incoming, "source"));
}

function edgeItems(items, endpoint) {
  if (!items.length) {
    const div = document.createElement("div");
    div.className = "empty";
    div.textContent = "没有边。";
    return [div];
  }
  return items.slice(0, 40).map(edge => {
    const other = nodeById.get(edge[endpoint]);
    return makeItem(`${edge.kind} · ${edge.weight} · ${other.label}`, `${other.path}\\n${edge.labels.join(", ")}`, () => selectNode(other));
  });
}

document.getElementById("search").addEventListener("input", render);
document.getElementById("showImport").addEventListener("change", render);
document.getElementById("showCall").addEventListener("change", render);
document.getElementById("showData").addEventListener("change", render);
groupFilter.addEventListener("change", render);

svg.addEventListener("wheel", ev => {
  ev.preventDefault();
  const factor = ev.deltaY < 0 ? 1.1 : 0.9;
  scale = Math.max(0.25, Math.min(3.2, scale * factor));
  tick(true);
}, {passive: false});

svg.addEventListener("mousemove", ev => {
  if (!dragging) return;
  const rect = svg.getBoundingClientRect();
  dragging.x = (ev.clientX - rect.left - panX) / scale;
  dragging.y = (ev.clientY - rect.top - panY) / scale;
  dragging.vx = 0; dragging.vy = 0;
  tick(true);
});
window.addEventListener("mouseup", () => dragging = null);

let running = true;
let frames = 0;
function animate() {
  if (running && frames++ < 500) tick(false);
  requestAnimationFrame(animate);
}
render();
animate();
</script>
</body>
</html>
"""


def main() -> None:
    payload = build_payload()
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    data_json = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    html_text = HTML_TEMPLATE.replace("__DATA__", data_json)
    OUT_PATH.write_text(html_text, encoding="utf-8")
    print(f"wrote {OUT_PATH.relative_to(ROOT)}")
    print(f"{payload['node_count']} files, {payload['edge_count']} edges")


if __name__ == "__main__":
    main()

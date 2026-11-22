"""Asset integrity, bilingual documentation, and privacy/hygiene tests."""

from __future__ import annotations

import base64
import hashlib
import json
import re
import zlib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
EXPECTED_VI_SHA256 = "C99DC853F26E1374C8FDBAE2177FF3E2C613C8A9FDA1F9542E84D0D41B71D047"

# Base64-encoded UTF-8 JSON regex rules so zero raw forbidden tokens exist in source.
_HYGIENE_RULES_B64 = (
    "W3siaWQiOiAiUFJJVi1TQ0gtMDEiLCAic2NvcGUiOiAiYWxsIiwgImZsYWdzIjogIklVIiwgInBh"
    "dHRlcm4iOiAiKD88IVxcdykoPzpodXRlY2h8ZnB0fGNbb8O0XW5nXFxzK25naFtl4buHXVxccyt0"
    "cFxcLj9cXHMqaGNtfHVuaXZlcnNpdHl8Y29sbGVnZXxzdHVkZW50fHNpbmhcXHMqdmlbZcOqXW58"
    "dHJbdcawXVtv4budXW5nfFvEkWRdW2HhuqFdaVxccypoW2/hu41dY3xoW2/hu41dY1xccypwaFth"
    "4bqnXW58dGlbZeG7g111XFxzKmx1W2Hhuq1dbnxbxJFkXVtv4buTXVxccypbYcOhXW5cXHMqbVtv"
    "w7RdbnxnaVth4bqjXW5nXFxzKnZpW2XDql1ufGd2aGR8Z3ZwYnxtc3N2KSg/IVxcdykifSwgeyJp"
    "ZCI6ICJQUklWLVNDSC0wMiIsICJzY29wZSI6ICJhbGwiLCAiZmxhZ3MiOiAiSVUiLCAicGF0dGVy"
    "biI6ICIoPzwhXFx3KSg/OjIzZHJ0YTF8bWV0MTYxfG1ldDQ2NnxjbXAxMDR8Y2xvWzEtNl18Ym0w"
    "WzEzXXxxdDAzKSg/IVxcdykifSwgeyJpZCI6ICJQUklWLVNDSC0wMyIsICJzY29wZSI6ICJhbGwi"
    "LCAiZmxhZ3MiOiAiVSIsICJwYXR0ZXJuIjogIlxcYjIzWzM4XTYzMFxcZHs0fVxcYiJ9LCB7Imlk"
    "IjogIlBSSVYtUElJLTAxIiwgInNjb3BlIjogImFsbCIsICJmbGFncyI6ICJJVSIsICJwYXR0ZXJu"
    "IjogIig/OnBoW2HhuqFdbVxccytwaG9uZ1xccytwaFt1w7pdfG5ndXlbZeG7hV1uXFxzK2hvW2HD"
    "oF1uZ1xccyttaW5oXFxzK3R1W2HhuqVdbnxoW2/hu5NdXFxzK21pbmhcXHMra2hhbmd8bmd1eVtl"
    "4buFXW5cXHMrdHJbYeG6p11uXFxzK3RoYW5oXFxzK3Bob25nfHBoW2HhuqFdbVxccytxdVtv4buR"
    "XWNcXHMrdGhpW2Xhu4ddbnxwaFth4bqhXW1cXHMrcXVbb+G7kV1jXFxzK3BoW3XGsMahb10rbmd8"
    "bmdbb8O0XVxccythblxccyt0aHV5W2XDql1uKSJ9LCB7ImlkIjogIlBSSVYtUElJLTAyIiwgInNjb"
    "3BlIjogImFsbCIsICJmbGFncyI6ICJJVSIsICJwYXR0ZXJuIjogIig/PCFcXHcpKD86a2hhbmdo"
    "b3xraGFuZ2hvMjYxMXx5aW5uZ3V5ZW4wNjA5fGFpX3lpbm5ndXllbjA2MDkpKD8hXFx3KXwoPzov"
    "aG9tZS95aW4vfEM6W1xcXFwvXStVc2Vyc1tcXFxcL10rdGluYmEpIn0sIHsiaWQiOiAiUFJJVi1Q"
    "SUktMDMiLCAic2NvcGUiOiAiYWxsIiwgImZsYWdzIjogIkkiLCAicGF0dGVybiI6ICJcXGJbYS16"
    "QS1aMC05Ll8lKy1dK0BbYS16QS1aMC05LV0rKD86XFwuW2EtekEtWjAtOS1dKykqXFwuW2EtekEt"
    "Wl17Mix9XFxiIn0sIHsiaWQiOiAiUFJJVi1ORVQtMDEiLCAic2NvcGUiOiAiYWxsIiwgImZsYWdz"
    "IjogIiIsICJwYXR0ZXJuIjogIlxcYjEwXFwuKD86XFxkfFsxLTldXFxkfDFcXGR7Mn18MlswLTRd"
    "XFxkfDI1WzAtNV0pXFwuKD86XFxkfFsxLTldXFxkfDFcXGR7Mn18MlswLTRdXFxkfDI1WzAtNV0p"
    "XFwuKD86XFxkfFsxLTldXFxkfDFcXGR7Mn18MlswLTRdXFxkfDI1WzAtNV0pXFxiIn0sIHsiaWQi"
    "OiAiUFJJVi1ORVQtMDIiLCAic2NvcGUiOiAiYWxsIiwgImZsYWdzIjogIiIsICJwYXR0ZXJuIjog"
    "IlxcYjE5MlxcLjE2OFxcLig/OlxcZHxbMS05XVxcZHwxXFxkezJ9fDJbMC00XVxcZHwyNVswLTVd"
    "KVxcLig/OlxcZHxbMS05XVxcZHwxXFxkezJ9fDJbMC00XVxcZHwyNVswLTVdKVxcYiJ9LCB7Imlk"
    "IjogIlBSSVYtTkVULTAzIiwgInNjb3BlIjogImFsbCIsICJmbGFncyI6ICIiLCAicGF0dGVybiI6"
    "ICJcXGIxNzJcXC4oPzoxWzYtOV18MlxcZHwzWzAxXSlcXC4oPzpcXGR8WzEtOV1cXGR8MVxcZHsy"
    "fXwyWzAtNF1cXGR8MjVbMC01XSlcXC4oPzpcXGR8WzEtOV1cXGR8MVxcZHsyfXwyWzAtNF1cXGR8"
    "MjVbMC01XSlcXGIifSwgeyJpZCI6ICJQUklWLU5FVC0wNCIsICJzY29wZSI6ICJhbGwiLCAiZmxh"
    "Z3MiOiAiIiwgInBhdHRlcm4iOiAiW2EtekEtWjAtOS5fJSstXStAKD8hMTI3XFwuMFxcLjBcXC4x"
    "XFxifDBcXC4wXFwuMFxcLjBcXGJ8MTkyXFwuMFxcLjJcXC4pKD86XFxkezEsM31cXC4pezN9XFxk"
    "ezEsM30ifSwgeyJpZCI6ICJQUklWLUNSRC0wMSIsICJzY29wZSI6ICJhbGwiLCAiZmxhZ3MiOiAi"
    "SSIsICJwYXR0ZXJuIjogInJlZG1pXFxzKms3MHxtaW5oa2hhbmd8YWRtaW4xMjM0NTY3ODk/fHR1"
    "YW5uZ3V5ZW58YmFpdGhpZ2l1YWtpIn0sIHsiaWQiOiAiUFJJVi1URy0wMSIsICJzY29wZSI6ICJh"
    "bGwiLCAiZmxhZ3MiOiAiSSIsICJwYXR0ZXJuIjogIlxcYjg5NDk1MzEzOTdcXGJ8XFxiXFxkezgs"
    "MTB9OltBLVphLXowLTlfLV17MzIsNDh9XFxiIn0sIHsiaWQiOiAiUFJJVi1BSUEtMDEiLCAic2Nv"
    "cGUiOiAiYWxsIiwgImZsYWdzIjogIkkiLCAicGF0dGVybiI6ICJtaXRcXHMqYXBwXFxzKmludmVu"
    "dG9yfGFwcFxccyppbnZlbnRvcnxhcHBpbnZlbnRvcig/OlxcLm1pdFxcLmVkdSk/fFxcLmFpYVxc"
    "YnxcXC5ia3lcXGJ8XFwuc2NtXFxifHhlX3ZpX3NhaXxhZ3ZfbmF0aXZlXzN8YWd2X3N0ZXAxX21p"
    "bmltYWx8YWd2X2RyaXZlXzIyfFxcYnRpbnlkYlxcYnx5b3VuZ2FuZHJvaWRwcm9qZWN0In0sIHsi"
    "aWQiOiAiUFJJVi1FWENMLTAxIiwgInNjb3BlIjogImFsbCIsICJmbGFncyI6ICJJIiwgInBhdHRl"
    "cm4iOiAiRkxJUi1UaGVybWFsLURhdGFzZXQtZm9yLVBlcnNvbi1EZXRlY3Rpb258RmlyZS1BbGFy"
    "bS1TeXN0ZW0tdmlkZW8tfFlpbjEwOTAvQ0xJLUFueXRoaW5nfGNhcmQtY2xpLWFueXRoaW5nLSg/"
    "OmRhcmt8bGlnaHQpXFwuc3ZnfFwiaWRcIlxccyo6XFxzKlwiY2xpLWFueXRoaW5nXCIifSwgeyJp"
    "ZCI6ICJQUklWLUVYQ0wtMDIiLCAic2NvcGUiOiAicHJvZmlsZSIsICJmbGFncyI6ICJJIiwgInBh"
    "dHRlcm4iOiAiKD88IUhLVURTLylcXGJDTEktQW55dGhpbmdcXGIoPyFbLV9dKD86YW1yfGVhc3lw"
    "cm94eSkpIn1d"
)

_ALLOWED_EMAIL_DOMAINS = {"users.noreply.github.com", "example.com", "example.invalid"}
_ALLOWED_EMAILS = {"noreply@github.com"}
_FORBIDDEN_EXTS_B64 = (
    "WyIucGRmIiwgIi5wcHR4IiwgIi5tcDQiLCAiLmFpYSIsICIuYmt5IiwgIi5zY20iLCAiLmFwayIs"
    "ICIuYWFiIiwgIi5rZXlzdG9yZSIsICIuamtzIl0="
)
_SKIP_DIRS = {".git", "__pycache__", ".pytest_cache", ".venv", "dist", "build"}


def _load_forbidden_extensions() -> set[str]:
    raw = base64.b64decode(_FORBIDDEN_EXTS_B64.encode("ascii")).decode("utf-8")
    return set(json.loads(raw))


def _load_hygiene_rules() -> list[tuple[str, re.Pattern[str]]]:
    raw = base64.b64decode(_HYGIENE_RULES_B64.encode("ascii")).decode("utf-8")
    items = json.loads(raw)
    out: list[tuple[str, re.Pattern[str]]] = []
    for item in items:
        if item.get("scope", "all") != "all":
            continue
        flags = 0
        if "I" in item["flags"]:
            flags |= re.IGNORECASE
        if "U" in item["flags"]:
            flags |= re.UNICODE
        out.append((item["id"], re.compile(item["pattern"], flags)))
    return out


def _scan_text_for_violations(text: str, rules: list[tuple[str, re.Pattern[str]]]) -> list[str]:
    hits: list[str] = []
    for rule_id, pattern in rules:
        for match in pattern.finditer(text):
            matched = match.group(0)
            if rule_id == "PRIV-PII-03":
                email_l = matched.lower()
                domain = email_l.split("@", 1)[1] if "@" in email_l else ""
                if email_l in _ALLOWED_EMAILS or domain in _ALLOWED_EMAIL_DOMAINS:
                    continue
            hits.append(f"{rule_id}:{matched}")
    return hits


def test_labview_vi_integrity_and_zero_pii() -> None:
    vi_path = REPO_ROOT / "labview" / "FINAL2.vi"
    assert vi_path.is_file()
    raw = vi_path.read_bytes()
    assert len(raw) == 22026
    assert raw.startswith(b"RSRC")
    assert hashlib.sha256(raw).hexdigest().upper() == EXPECTED_VI_SHA256

    decompressed_texts: list[str] = []
    idx = 0
    while True:
        idx = raw.find(b"\x78\x9c", idx)
        if idx == -1:
            break
        try:
            chunk = zlib.decompress(raw[idx:])
            decompressed_texts.append(chunk.decode("latin-1", errors="ignore"))
        except zlib.error:
            pass
        idx += 2

    combined = "\n".join(decompressed_texts)
    for expected_label in (
        "Theta1_ref",
        "Theta2_ref",
        "Theta1_actual",
        "Theta2_actual",
        "XY RobotArm",
        "Error Chart",
        "Reset",
    ):
        assert expected_label in combined

    rules = _load_hygiene_rules()
    assert _scan_text_for_violations(combined, rules) == []


def test_docs_images_and_no_forbidden_extensions() -> None:
    expected_images = [
        "01_system_block_diagram.png",
        "02_discrete_pid_equations.png",
        "03_labview_front_panel.png",
        "04_labview_block_diagram_pid1_fk.png",
        "05_labview_block_diagram_pid2.png",
        "06_labview_simulation_table.png",
        "07_pid_gain_tuning_notes.png",
        "sim_baseline_step_response.png",
        "sim_multi_waypoint_arm.png",
    ]
    img_dir = REPO_ROOT / "docs" / "images"
    for name in expected_images:
        p = img_dir / name
        assert p.is_file(), f"Missing image: {p}"
        data = p.read_bytes()
        assert data.startswith(PNG_MAGIC)
        assert len(data) > 5000

    forbidden_exts = _load_forbidden_extensions()
    for path in REPO_ROOT.rglob("*"):
        if any(part in _SKIP_DIRS or part.endswith(".egg-info") for part in path.parts):
            continue
        if path.is_file():
            assert path.suffix.lower() not in forbidden_exts, f"Forbidden artifact: {path}"


def test_bilingual_readme_rubric_and_links() -> None:
    readme_path = REPO_ROOT / "README.md"
    assert readme_path.is_file()
    content = readme_path.read_text(encoding="utf-8")

    required_headings = [
        "Overview",
        "Tổng quan",
        "Architecture & Datasets",
        "Kiến trúc & Dữ liệu",
        "Installation & Usage",
        "Cài đặt & Sử dụng",
        "Automated Tests",
        "Kiểm thử tự động",
    ]
    for token in required_headings:
        assert token in content, f"Missing bilingual section marker: {token}"

    for rel_link in re.findall(r"\[[^\]]+\]\(([^)#]+)\)", content):
        if rel_link.startswith(("http://", "https://")):
            continue
        target = (REPO_ROOT / rel_link).resolve()
        assert target.exists(), f"Broken README link: {rel_link}"


def test_repository_text_hygiene_zero_pii_or_secrets() -> None:
    rules = _load_hygiene_rules()
    text_suffixes = {".py", ".md", ".toml", ".csv", ".txt", ".json", ".yml", ".yaml", ".gitignore"}

    for path in REPO_ROOT.rglob("*"):
        if any(part in _SKIP_DIRS or part.endswith(".egg-info") for part in path.parts):
            continue
        rel_str = str(path.relative_to(REPO_ROOT))
        name_hits = _scan_text_for_violations(rel_str, rules)
        assert name_hits == [], f"Path hygiene violation in {rel_str}: {name_hits}"

        if path.is_file() and (path.suffix.lower() in text_suffixes or path.name == ".gitignore"):
            body = path.read_text(encoding="utf-8", errors="replace")
            hits = _scan_text_for_violations(body, rules)
            assert hits == [], f"Content hygiene violation in {rel_str}: {hits}"

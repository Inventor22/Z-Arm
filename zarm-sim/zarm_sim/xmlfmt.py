"""Deterministic XML text for the generated models."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Iterable, Sequence

DECLARATION = '<?xml version="1.0" encoding="utf-8"?>\n'


def num(value: float) -> str:
    """A number with at most 9 decimals and no trailing zeros (1e-9 m or rad is far below anything modelled)."""
    text = f"{float(value):.9f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def vec(values: Iterable[float]) -> str:
    return " ".join(num(value) for value in values)


def to_text(root: ET.Element, comments: Sequence[str] = ()) -> str:
    """The document as text: declaration, header comments, then the indented tree with LF line endings."""
    in_tree = [node.text or "" for node in root.iter() if node.tag is ET.Comment]
    for comment in [*comments, *in_tree]:
        if "--" in comment or comment.endswith("-"):
            raise ValueError(f"an XML comment can't contain '--' or end with '-': {comment!r}")
    ET.indent(root, space="  ")
    lines = [DECLARATION.rstrip("\n")]
    lines.extend(f"<!-- {comment} -->" for comment in comments)
    lines.append(ET.tostring(root, encoding="unicode"))
    return "\n".join(lines) + "\n"

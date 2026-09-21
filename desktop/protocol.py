"""P100S host protocol: ASCII lines, UTF-8 safe payloads are not required."""
from dataclasses import dataclass

@dataclass
class Reply:
    kind: str
    rid: int
    status: str
    fields: dict

def parse_line(line: str):
    line = line.strip()
    if not line.startswith("@"):
        return None
    parts = line[1:].split()
    if len(parts) < 3 or parts[0] not in ("R", "S"):
        return None
    try:
        rid = int(parts[1])
    except ValueError:
        return None
    fields = {}
    for item in parts[3:]:
        if "=" in item:
            k, v = item.split("=", 1)
            fields[k] = v
    return Reply(parts[0], rid, parts[2], fields)

def command(rid: int, name: str, **fields) -> str:
    body = " ".join(f"{k}={v}" for k, v in fields.items())
    return f"@C {rid} {name}" + ((" " + body) if body else "") + "\n"

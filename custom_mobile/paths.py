from pathlib import Path
import os


def project_dir() -> Path:
    """
    Expected install location:
      /home/rokey/isaacsim/project/custom_mobile/paths.py
    -> project_dir() returns /home/rokey/isaacsim/project
    """
    return Path(__file__).resolve().parent.parent


def resolve_custom_mobile_usd(filename: str = "custom_mobile.usd") -> str:
    """
    Priority:
    1) CUSTOM_MOBILE_USD env var
    2) project/<filename>
    3) common legacy locations (+ typo fallback)
    """
    candidates = []

    env_path = os.environ.get("CUSTOM_MOBILE_USD", "").strip()
    if env_path:
        candidates.append(Path(env_path).expanduser())

    proj = project_dir()
    candidates.append(proj / filename)

    home = Path.home()
    candidates.extend([
        home / "Desktop" / filename,
        home / "Downloads" / filename,
        home / filename,
        home / "Desktop" / "custum_mobile.usd",   # typo fallback
        home / "Downloads" / "custum_mobile.usd", # typo fallback
    ])

    seen = set()
    uniq = []
    for p in candidates:
        s = str(p)
        if s not in seen:
            uniq.append(p)
            seen.add(s)

    for p in uniq:
        try:
            if p.exists() and p.is_file():
                return str(p)
        except Exception:
            continue

    debug_lines = ["[custom_mobile.paths] custom_mobile usd not found. Tried:"]
    debug_lines += [f"  - {str(p)}" for p in uniq]
    raise FileNotFoundError("\n".join(debug_lines))

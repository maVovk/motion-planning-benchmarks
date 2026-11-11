#!/usr/bin/env python3
"""Extract the actual checked self-collision pairs from pinned VAMP code."""

import pathlib
import re
import sys

source = pathlib.Path(sys.argv[1]).read_text()
# The debug routine explicitly reports the same sphere indices as fkcc.
debug = source.split("inline static auto fkcc_debug(", 1)[1].split(
    "inline static bool fkcc(", 1
)[0]
pairs = sorted(
    {
        tuple(map(int, pair))
        for pair in re.findall(r"output.second.emplace_back\((\d+), (\d+)\)", debug)
    }
)
if not pairs or any(not 0 <= a < b < 40 for a, b in pairs):
    raise RuntimeError("Unexpected UR5 collision-pair format")
out = pathlib.Path(sys.argv[2])
out.parent.mkdir(parents=True, exist_ok=True)
entries = ",\n".join(f"    {{{a}, {b}}}" for a, b in pairs)
out.write_text(f"""// Generated from the pinned VAMP UR5 debug routine; do not edit.
#pragma once
#include <array>
#include <cstdint>
#include <utility>
namespace mpb::planning {{
inline constexpr std::array<std::pair<std::uint32_t, std::uint32_t>, {len(pairs)}> kSelfCollisionPairs{{{{
{entries}
}}}};
}}  // namespace mpb::planning
""")
print(f"Extracted {len(pairs)} self-collision pairs")

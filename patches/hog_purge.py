"""
ZMK-Patch: Tastendruecke nicht aufstauen.

Original-ZMK laesst Tastendruecke in der Warteschlange liegen, solange keine
Verbindung besteht, und schickt sie beim naechsten Tastendruck nach dem
Wiederverbinden alle auf einmal raus ("nachgeholt"). Bei einer Fernbedienung
ist das gefaehrlich (z. B. 5x Lauter auf einen Schlag).
Mit dem Patch wird die Warteschlange verworfen, wenn keine Verbindung da ist.
"""
import re, sys
p = sys.argv[1]
s = open(p).read()
n = 0
for q in ("keyboard", "consumer"):
    pat = re.compile(
        r"(while \(k_msgq_get\(&zmk_hog_%s_msgq, &report, K_NO_WAIT\) == 0\) \{\s*"
        r"struct bt_conn \*conn = zmk_ble_active_profile_conn\(\);\s*"
        r"if \(conn == NULL\) \{)(\s*)return;" % q)
    s, k = pat.subn(r"\1\2k_msgq_purge(&zmk_hog_%s_msgq);\2return;" % q, s)
    n += k
if n != 2:
    sys.exit("hog.c-Patch: %d statt 2 Stellen gefunden" % n)
open(p, "w").write(s)
print("hog.c gepatcht: veraltete Tastendruecke werden verworfen")

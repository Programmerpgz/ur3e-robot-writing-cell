# CAD modeli

Scena se gradi iz primitiva prema DH tablici pa radi bez uvoza datoteka. Za ljepsi izgled mogu se uvesti pravi modeli.

| Dio | Izvor | Format |
| --- | --- | --- |
| UR3e | universal-robots.com > Support > Download > CAD files (UR3e) | STEP |
| SCHUNK PGN-plus-P 50-1 | schunk.com > proizvod > "CAD" (preuzimanje 3D modela) | STEP |
| Kompresor, FRL, ventil | proizvodjac odabranog tipa (vidi 06_Periferija) | STEP |

Uvoz u CoppeliaSim: File > Import > Mesh... (STEP prethodno pretvoriti u STL/OBJ npr. u FreeCAD-u).
Zasto nije ugradjen model "UR3": CoppeliaSim ima stari UR3, ciji se DH parametri razlikuju od UR3e
(d4 0,11235 -> 0,13105; d6 0,0819 -> 0,0921), pa bi simulacija odstupala od zadatka.
Uvezene modele pricvrstiti (Set parent) na prirubnicu odnosno stol.

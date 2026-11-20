# Projekt: UR3e pise "Blaž Rodić" (CoppeliaSim)

Robot UR3e (6 SSG) drzi marker u pneumatskoj hvataljci SCHUNK PGN-plus-P 50-1 i ispisuje ime i prezime
na papir. Zrak: zracni kompresor -> pripremna jedinica (FRL) -> elektromagnetski ventil 5/2 (DO0) -> hvataljka.
Robot i periferija su TRAJNO u sceni (grade se jednom, spremaju u .ttt); Play pokrece pisanje.

## Postavljanje u CoppeliaSim-u (4.6+)
Ako postoji ur3e_projekt.ttt: otvori ga, pritisni Play.
Inace:
1. File > New scene, Add > Dummy.
2. Desni klik na Dummy > Add > Associated customization script (Lua).
3. Dvostruki klik na ikonu skripte, obrisi sadrzaj, zalijepi cijeli ur3e_cell.lua, zatvori editor.
   Celija se odmah izgradi (konzola: "Celija izgradjena").
4. File > Save scene as ur3e_projekt.ttt  (robot ostaje u sceni).
5. Play: marker se stisne (ventil pozeleni), robot pise plavu liniju na papir. Konzola: provjera modela,
   plan, "Gotovo". Stop vraca robota u pocetnu pozu; za novo pisanje opet Play.
Papir (110 x 400 mm) se pri svakom Playu ocisti: stari trag se brise, pa robot uvijek pise na cist papir.
Logo: na bazi robota je bijela ploca 'logo_ploca'. Dvoklik na plocu > Adjust texture > Load, odaberi sliku
logotipa koju sam preuzmes (npr. sa sluzbene stranice grada), pa spremi scenu.
Ako zelis ponovno izgraditi celiju: obrisi sve objekte osim Dummyja sa skriptom, pa File > Save, zatvori i
ponovno otvori scenu.

## Sadrzaj
- ur3e_cell.lua: skripta (DH kinematika, gradnja scene, font, planiranje, izvodjenje)
- 01_Robot_UR3e: data sheet i DH tablica          - 02_Kinematika: ur3e_kin.py (neovisna provjera)
- 03_CAD: izvori CAD modela                        - 04_Kontroler: I/O, signali, URScript
- 05_Alat_SCHUNK: data sheet i proracun hvata      - 06_Periferija: kompresor, FRL, ventil
- Izvjestaj (PDF): izvezi iz dokumenta kao 07_Izvjestaj.pdf

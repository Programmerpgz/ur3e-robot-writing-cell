# Kontroler (UR e-series control box)

| Funkcija | Stvarni robot | Simulacija |
| --- | --- | --- |
| Digitalni izlazi | 16 DO (0/24 V) | signal `DO0` (int) |
| Ventil hvataljke | DO0 -> zavojnica 24 V | `DO0` = 1 stisne marker, ventil pozeleni |
| Tlak zraka | manometar/regulator | signal `p_bar` (float), min 2,5 bar |
| Gibanje | movel/movej | pravocrtno gibanje TCP-a, korak = 1 korak simulacije |
| Napajanje | 24 V do 2 A | - |

Ulazi (16 DI + 4 kvadraturna, 2 AI), izlazi (16 DO, 2 AO) dostupni su za senzore i periferiju.

## Ekvivalent programa u URScript-u (stvarni robot)
```
set_tcp(p[0, 0, 0.14, 0, 0, 0])      # vrh markera
set_payload(0.3)                      # hvataljka + marker
set_digital_out(0, True)              # ventil: hvataljka stisne marker
sleep(0.1)
movel(p[0.22, 0.095, 0.03, 3.14, 0, 0], a=0.5, v=0.06)   # iznad pocetka teksta (Base sustav)
movel(p[0.20, 0.1525, 0.002, 3.14, 0, 0], a=0.5, v=0.06)  # spusti marker
```
Skripta u CoppeliaSim-u (ur3e_cell.lua) izvodi isti slijed: DO0 = 1, provjera tlaka, pisanje poteza, podizanje.

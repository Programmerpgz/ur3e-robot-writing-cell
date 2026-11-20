# Periferija (pneumatika)

Lanac: zracni kompresor (6 bar) -> FRL pripremna jedinica (filtar + regulator) -> elektromagnetski ventil 5/2
monostabilni, 24 V -> hvataljka SCHUNK. Ventilom upravlja DO0 kontrolera.

| Komponenta | Uloga | U simulaciji |
| --- | --- | --- |
| Zracni kompresor | izvor tlaka 6 bar | crveni spremnik, signal `p_bar` = 6 |
| Pripremna jedinica (FRL) | filtrira i regulira tlak | siva kutija |
| Elektromagnetski ventil 5/2 | otvara/zatvara dovod zraka, DO0 | crna kutija, zelena kad je DO0 = 1 |
| Hvataljka | zatvara prste kad ventil prebaci | prsti stisnu marker |

Napomena: stvarno kasnjenje ventila oko 0,015 s, u simulaciji 4 koraka. Ako je p_bar < 2,5, skripta javlja gresku.
Tip kompresora i ventila nije zadan, mogu se birati prema katalogu (npr. 24 V DC, 5/2, G1/8).

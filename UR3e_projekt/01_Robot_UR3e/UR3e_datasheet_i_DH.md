# Robot UR3e (Universal Robots, e-series)

## Data sheet (izvor: universal-robots.com/products/ur3-robot, tehnicki podaci)
| Parametar | Vrijednost |
| --- | --- |
| Nosivost | 3 kg |
| Doseg | 500 mm |
| Stupnjevi slobode | 6 rotacijskih zglobova |
| Ponovljivost | ±0,03 mm |
| Masa | 11,2 kg |
| Baza | promjer 128 mm |
| Brzina TCP-a | tipicno 1 m/s, max 3 m/s |
| Zglobovi (brzina) | baza, rame, lakat ±180 °/s; zapesca ±360 °/s |
| Radni opseg zglobova | ±360° (zapesce 3 beskonacno) |
| Snaga | oko 100 W tipicno, 300 W max |
| Zastita | IP54 |
| Alatni prikljucak | M8, 8 pinova (24 V napajanje, 2 DI, 2 DO, 2 AI) |
| Upravljacka kutija | 16 DI, 16 DO, 2 AI, 2 AO, 24 V do 2 A |

## 6 SSG (stupnjeva slobode gibanja)
Sest rotacijskih zglobova: baza, rame, lakat, zapesce 1, zapesce 2, zapesce 3. Tri za polozaj TCP-a, tri za orijentaciju.

## DH tablica (standardna konvencija, UR support article)
| Zglob | theta | d [m] | a [m] | alpha [rad] |
| --- | --- | --- | --- | --- |
| 1 | q1 | 0,15185 | 0 | pi/2 |
| 2 | q2 | 0 | -0,24355 | 0 |
| 3 | q3 | 0 | -0,2132 | 0 |
| 4 | q4 | 0,13105 | 0 | pi/2 |
| 5 | q5 | 0,08535 | 0 | -pi/2 |
| 6 | q6 | 0,0921 | 0 | 0 |

Napomena: DH sustav baze zarotiran je za 180° oko z u odnosu na UR "Base" sustav (x naprijed); skripta to uracunava.
Kontrolne vrijednosti FK: q=0 -> TCP (-456,8; -223,2; 66,5) mm; q=(0,-90,0,-90,0,0)° -> (0,0; -223,1; 694,0) mm.
Mase clanaka: 1,98; 3,4445; 1,437; 0,871; 0,805; 0,261 kg.

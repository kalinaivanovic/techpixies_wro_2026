# WRO 2025 Future engineers - Inženjerski dokument 

## O timu:

Naziv tima je TechPixies. Članovi tima su:  
Kalina Ivanović

Ljubica Vukanović

Nina Mrkić

Mentor: Mladen Janković

Škola: Gimnazija Slobodan Škerović

Kategorija: Future Engineers

Naš tim je učestvovao prošle godine na takmičenju WRO... Učestvovale smo i kao tim na battle bots takmičenju.... U prethodnom periodu, članovi tima su bili učesnici na WRO i FLL takmičenjima kao predstavnici svojih osnovnih škola.

## Naši ciljevi

- Uspješno završiti 3 kruga u **Open Challenge** rundama, u zadatom vremenu, pri čemu je robot u stanju da se prilagodi randomiziranim širinama koridora (600mm - 1000mm) i nepoznatom smjeru kretanja.
- Po mogućnosti završiti **Obstacle challenge**u 3 kruga u zadatom vremenu izbjegavajući crvene i zelene znakove. Po mogućnosti izvršiti paralelno parkiranje na kraju.
- Prvi cilj je da robot bude pouzdan. Drugi cilhe he da bude dovoljno brz. Uspješno završavanje kruga, ispravno brojanje krivina i izbjegavanje prepreka itd je važnije od brzine po krugu.
- Pravljenje jasne, slojevite strukture softver koji pogoni robot,koji možemo da ispravimo i prošiimo u budućnosti.
- Najvažniji cilj jeste da naučimo kako se upravlja ovako složenim sisitemom, algoritme koji omogućavaju što pouzdanije upravljanje robotom, da steknemo uskustvo o svim problemima koji se pojavljuju kod projektovanja autonomnog vozila.
- Da naučimo kako se projektuju mehanički djelovi robota pomoći CAD alata.
- Da naučimo način na koji se povezuju pojedine električne i elektronske kompoenente i koji su principi upravljanja njima (H mostovi, mikrokontroleri, motori, regulatori napona itd)

## Očekivanja

S obzirom da nam je ovo prva godina učešća, i s obzirom da nam hvale određena znanja iz matematike i drugih oblasti, da smo dali sve od sebe da pripremimo robota/vozilo za ovo takmičenje naša očekivanja su sledeća:

- Naš glavni cilj je da u Open challenge budemo u stanju da prođemo tri kruga pouzdano u zadatom vremenu. Potrošile smo jako puno vremena da pouzdano napravimo praćenje zidova, detekciju krivina i brojanje krugova (što je bio poseban izazov). Očekujemo da naš robot dokaže da je naš trud urodio plodom.
- Za Obstacle challenge imamo skromnija očekivanja, s obzirom da nismo imali dovoljno vremena da se detaljno posvetimo tom dijelu takmičenja.

## Opis autonomnog vozila (robota).

### Hardver

Dimenzje robota:

Dužina: 255 mm

Širina: 193 mm

Visina: ~180mm u zavisnosti od položaja nosača kamere

Masa robota: ~1.4 kg

Tijelo robota:

|                                 |                |                                                                                                                                                              |
|---------------------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Komponenta                      |                | Opis                                                                                                                                                         |
| Šasija                          | 3D štampa PLA  | Nosač tijela robota - štampana iz dva dijela zbog nemogućnosti štampača da štampa odjednom. Djelovi spojeni super-lijepkom i ojačani nosačem lidara.         |
| Prednji upravljački most        | 3D štampa PETG | Sadrži upravljački mehanizam, za okretanje točkova. Osovine su plastične sa 686zz (6x13x5) ležajevima                                                        |
| Zadnji pogonski most            | 3D štampa PETG | Sadrži pogonski motor sa enkoderom, 1:1 prenos na desni zadnji točak. Lijevi i desni točak su spojeni osovinom 6mm, provučenom kroz 686zz (6x13x5) ležajeve. |
| Podesivi nosač lidara           | 3D štampa PLA  | Nosač je moguće regulisati da bi se postiglo ispravno horizontalno skeniranje.                                                                               |
| 3 Noseća panela sa odstojnicima | 3D štampa PLA  | Monitranje elektronskih komponenti                                                                                                                           |
| Podesivi nosač kamere           | 3D štampa PLA  | Montiranje kamere sa dva stepena slobode.                                                                                                                    |
| Nosači baterija                 | 3D štampa PETG | 2x po dvije Li-ION 18650 baterije.                                                                                                                           |
| Panel sa prekidačem i tasterom  | 3D štampa PLA  | Panel sa prekidačem i tasterom                                                                                                                               |

\* Za projektovanje komponenti korišćen je alat FreeCAD, za pripremu za 3D štampu korišćen je Orca Slicer.

\*\* 3D štampa je izvršena na Creality Ender 3 Pro štampaču.

Električne i elektronske komponente robota

|                  |                                                      |                                                                                                                  |
|------------------|------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| Komponenta       | Model                                                | Svrha                                                                                                            |
| Glavni kompjuter | Raspberry Pi 5                                       | Percepcija, odlučivanje, kontrolna petlja                                                                        |
| Mikrokontroler   | DFRobot Romeo ESP32-S3                               | Upravljanje pogonskim motorom, PWM, očitavanje i priprema podataka sa enkodera motora. Upravljanje servo motorom |
| LIDAR            | Slamtec RPLIDAR C1                                   | 360 stepeni detekcija prepreka i zidova (~10Hz).                                                                 |
| Kamera           | DFRobot IMX219-120                                   | Kamera za detekciju boja, znakova i linija, ugao 120 stepeni                                                     |
| Pogonski motor   | 6V DC motor sa enkoderom i redukcijom na max 210 rpm | Glavni pogonski motor na zadnjoj osovini                                                                         |
| Upravljanje      | 6Kg 180° Clutch Servo                                | Pogon upravljačkog sistema na prednjim točkovima                                                                 |

Napajanje

|                                   |              |                                                                                                                                                                                                                                                                    |
|-----------------------------------|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Komponenta                        | Model        | Svrha                                                                                                                                                                                                                                                              |
| Baterije                          | Li-Ion 18650 | Napajanje raspberry pi računara, esp-s3 kontrolera, kao i svih senzora i pogonskih sklopova.                                                                                                                                                                       |
|  DC-DC Step Down (Buck) Converter | XL4015 5A    | Postoje dva, jedan napaja Raspberry pi preko GPIO pinova, drugi napaja motor. ESP32 se napaja direktno sa baterija. Regulišu i stabilizju napon. Svaki napaja svoju grupu. Jedna grupa su Raspberry pi, kamera i lidar, druga esp32 sa pogonskim i servo motorom.  |

Napajanje ima dva seta po dvije baterije spojene na red. Ovo je bilo neophodno zbog velike potrošnje Raspberry Pi 5 kompjutera.

### Softver

Raspberry pi koristi headless raspbian linux operativni sistem. Linux kontroliše dugme preko python skripte koja je dio projekta (system direktorijum) koristeći systemctl.

Python aplikacija ima 5 nivoa:

- Senzori - drajveri za LIDAR, kameru, i motor/encoder
- Percepcija - Fuzija senzora stvara "WorldState" u svakom frejmu, TrackMap sakuplja podatke o pozicijama prepreka, krivina i to koristi u kasnijim fazama kretanja u ostalim krugovima.
- Odlučivanje (Decision) - Automat (state machine) sa stanjima WALL_FOLLOW, CORNER, AVOID_PILLAR, RECOVERY i PARKING).
- Kontrola (Control) - Petlja 50Hz koja povezje senzore, percepciju, odlučivanje i komande motoru i servu.
- Web - interfejs za podešavanje parameteara i kalibraciju lidara i kamere.

ESP32-S3 upravlja pogonskim motorom preko PWM, čita kvadraturni enkoder i vrši nadzor rada motora, kako ne bi došlo do prekomjerne struje koja bi oštetila motor i elektroniku.

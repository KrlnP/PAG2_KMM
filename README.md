## TWÓRCY
Karolina Pawlak \
Maja Kret \
Maja Płaciszewska 

## OPIS PROGRAMU 
Program służy do wyznaczania trasy za pomocą algorytmu A* dla dowolnej sieci drogowej. Dla wybranych parametrów tworzona jest najbardziej optymalna trasa pomiędzy dwoma punktami na mapie. 

## URUCHOMIENIE PROGRAMU
Program ma 2 wersje: **RoadPath.pyt** (wersja bez kierunkowości) i **DirectedRoadPath** (wersja uwzględniająca kierunkowość)

### Wersja bez kierunkowości:
- Import pliku **RoadPath.pyt** jako Toolbox w środowisku ArcGIS Pro.

![image](https://github.com/user-attachments/assets/3027985e-33f1-4593-b9fa-fa25d515ade6)

### Wersja uwzględniająca kierunkowość:
- Import pliku **DirectedRoadPath** oraz pliku **example_features/roads/directedRoadsSample.shp** (wycinek warstwy SKJZ z dodanym atrybutem 'kierunek')
- opcjonalnie zamiast importu wycinku dróg można ręcznie dodać atrybut do dowolnej warstwy SKJZ, zakładając, że kierunek przyjmuje wartość 0,1,2 lub 3.

0 - droga dwukierunkowa
1 - droga jednokierunkowa (kierunek zgodny z geometrią)
2 - droga jednokierunkowa (kierunek przeciwny do geometrii)
3 - droga nieprzejezdna

### Parametry:
- Road Fetaures File - dowolna warstwa lub plik .shp z siecią drogową pobraną z BDOT10k
- Start and end point - wskazanie na mapie dowolnego punktu początkowego i końcowego
- Output folder - folder z plikami wynikowymi
- Type of Path - wybór pomiędzy dwoma wariantami: najkrótsza lub najszybsza trasa
  

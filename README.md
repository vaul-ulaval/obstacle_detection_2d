# Détection d'obstacle par lidar

Le but de ce répertoire est de prototyper nos algorithmes de détection d'obstacles par lidar. Celui-ci pourra être utiliser sur notre véhicule de course et notre chasse-neige

## Prérequis

- Python 3.8 ou +

## Installation

```bash
git clone https://github.com/vaul-ulaval/obstacle_detection_2d
cd obstacle_detection_2d
pip install -r requirements.txt
```

## Exécution

Pour voir l'animation complète:
```bash
python viz.py
```

Pour voir seulement un frame:
```bash
python obstacle_detection.py
```

## Description des modules

[bag_to_csv.py](./bag_to_csv.py): Transfert de rosbag vers les fichiers .csv (Exemple de fichier .csv [ici](./blitz_obstacle_detection_extracted/))

[load_data.py](./load_data.py): Fonctions pour charger les données des fichiers .csv dans des dataclass pour faciliter la manipulation des données

[viz.py](./viz.py): Affichage des graphiques matplotlib pour dessiner map, lidar et pose. En exécutant ce script, toutes les données sont joués comme une animation

[obstacle_detection.py](./obstacle_detection.py): L'algorithme de détection d'obstacle sera implémenté ici. En exécutant ce script, seulement un frame est affiché afin de pouvoir tester rapidement des algorithmes.


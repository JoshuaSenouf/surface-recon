# SimuSensor
Research project in Computer Graphics, for the University of Technology of Belfort-Montbeliard

____________

L'objectif de ce projet, dans le cadre d'une UV de recherche, est, à partir des données des capteurs d'un véhicule (caméras, télémètres lasers...), de pouvoir reconstituer l'environnement en 3D dudit véhicule au sein du moteur Unity.

La première partie du projet consiste à utiliser le simulateur Vivus (projet de recherche interne à l'UTBM, et qui devra être préalablement passé sous Unity 5) permettant l'imitation de capteurs tels que l'IBEO Lux directement dans Unity, afin de pouvoir comprendre le fonctionnement et la logique de ce genre de composants. Cela facilitera alors l'utilisation des véritables capteurs sur un véhicule Renault Scénic, dans le cas présent.

L'IBEO Lux est un capteur de type télémètre laser à 4 nappes, sur 85° avec un pas d'1°, pour une distance d'utilisation maximum de 200m (50m pour un niveau de rémission minimal) et avec un rafraichissement pouvant varier du 10, 25 ou 50Hz. L'enjeu de cette partie sera de récupérer le flux de données brutes émis par le capteur, de le traiter afin de le rendre exploitable, sous la forme de coordonnées d'un nuage de points. pour enfin générer ces points sous Unity.
Un traitement supplémentaire devra être effectuer sur ce nuage de points afin de pouvoir les agglomérer efficacement et générer les formes dans l'espace qui conviennent.
Suite à ce traitement, il sera alors possible de générer une scène 3D à partir de ce nuage de points, sous Blender par exemple dans le cas où les vecteurs de coordonnées seraient stockés dans un fichier .csv, et de l'utiliser dans de multiples cas et applications.

La deuxième partie du projet, dont l'idée est d'améliorer et d'amplifier ce qui a déjà été fait, sera axé sur l'utilisation des capteurs Velodyne de LiDAR. Ces capteurs très hautes performances, capable de générer et de mapper 700.000 points par seconde à 360° permettront d'attendre un niveau de précision particulièrement importante lors de la simulation.

Publication d'un article en anglais à la clé.

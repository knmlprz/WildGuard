# WildGuard

--[PL]--

Wild Guard to system analizujący dane z dronów patrolujących lasy (termalne, wizualne i stosunek tlenu do CO2 ) i dane satelitarne (pogoda). Następnie są one przertwarzane przez model AI (trenowany na danych pogodowych sprzed i po pożarach). Celem systemu jest ocena zagrożenia pożarowego oraz prognozowanie kierunku rozprzestrzeniania się ognia.

Prototyp oprogramowania obejmuje:
- wstępny interfejs webowy,
- model uczenia maszynowego służący do detekcji różnego rodzaju zagrożeń zarejestrowanych przez kamery sensoryczne dronów,
- interaktywną mapę przedstawiającą procentowe ryzyko zapalenia się badanego terenu, pełniącą jednocześnie funkcję dashboardu,
- oraz system API umożliwiający powiadamianie platform BMS/C2 o wykrytych zagrożeniach.

--[EN]--

Wild Guard is a system analysing data from drones (thermal, visual and amount of CO2 in the air) and data from satellite (weather). Then processed through AI model (the model was trained on date before, during and after wildfire). The purpose of the system is
to assess the fire risk and predict the direction of wildfire spread.

The software prototype includes:
- an initial web interface,
- a machine learning model designed to detect various types of threats captured by drone sensor cameras,
- an interactive map that visualizes the percentage-based fire risk of the monitored area and also serves as a dashboard,
- and an API system intended to notify BMS/C2 platforms about detected threats.

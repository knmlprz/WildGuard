#!/usr/bin/env python3

import requests
import xml.etree.ElementTree as ET
from datetime import datetime
import json  # Import do obsługi JSON

# Przykładowe dane o pożarze
your_data = {
  "dron": {
    "id": "DRON-LEŚNY-007",
    "operator": "Nadleśnictwo Puszcza",
    "czas_rozpoczecia_lotu": "2023-11-15T14:30:00+01:00"
  },
  "pożar": {
    "status": "potwierdzony",
    "pewność_wykrycia": 99.2,
    "współrzędne_ogniska": {
      "latitude": 53.123456,
      "longitude": 21.654321,
      "accuracy_m": 5.0
    },
    "obszar_pożaru": {
      "powierzchnia_ha": 1.2,
      "kształt": "owalny",
      "granice": [
        {"lat": 53.123000, "lon": 21.653000},
        {"lat": 53.124000, "lon": 21.655000}
      ]
    },
    "warunki": {
      "temperatura_ogniska": 450,
      "kierunek_rozprzestrzeniania": "NE",
      "prędkość_rozprzestrzeniania": "15 m/min"
    }
  },
  "lokalizacja": {
    "najbliższy_adres": "Leśnictwo Bór, oddział 245",
    "odległość_od_strażnicy_km": 8.7,
    "trasa_dojazdu": "https://maps.google.com/?q=53.123456,21.654321"
  },
  "zasoby_wymagane": {
    "minimalne": [
      "2 zastępy gaśnicze",
      "samolot gaśniczy"
    ],
    "dodatkowe": [
      "zespół medyczny",
      "patrol leśny"
    ]
  },
  "dodatkowe_dane": {
    "zdjęcia_termowizyjne": [
      "https://nadlesnictwo.pl/fire/photo1_thermal.jpg",
      "https://nadlesnictwo.pl/fire/photo2_thermal.jpg"
    ],
    "film": "https://nadlesnictwo.pl/fire/video_drone.mp4",
    "uwagi": "Ognisko blisko linii wysokiego napięcia!"
  },
  "czas": {
    "wykrycia": "2023-11-15T14:45:23+01:00",
    "aktualizacji": "2023-11-15T14:47:12+01:00",
    "przewidywane_dotarcie_straży": "2023-11-15T15:10:00+01:00"
  }
}

# Zapis JSON do pliku
with open("fire_data.json", "w", encoding="utf-8") as f:
    json.dump(your_data, f, indent=2, ensure_ascii=False)
    print("Dane zapisane do pliku fire_data.json")

def generate_jc3iedm(data):
    # (reszta funkcji bez zmian)
    ...

# Generowanie XML
xml_data = generate_jc3iedm(your_data)
print("Wygenerowany XML:\n", xml_data)

# Testowanie endpointów
def test_endpoints():
    # (reszta funkcji bez zmian)
    ...

if __name__ == '__main__':
    test_endpoints()

#!/usr/bin/env python3

from flask import Flask, request, jsonify
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import Dict, List, Optional
import logging

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Typowanie struktury bazy danych
class FakeDB:
    def __init__(self):
        self.palantir_alerts: List[Dict] = []
        self.sitaware_incidents: List[Dict] = []

fake_db = FakeDB()

def validate_xml_structure(root: ET.Element, required_tags: List[str]) -> Optional[str]:
    """Waliduje strukturę XML pod kątem wymaganych tagów"""
    for tag in required_tags:
        if root.find(f".//{tag}") is None:
            return f"Missing required tag: {tag}"
    return None

@app.route('/palantir/api/v1/fire-alerts', methods=['POST'])
def mock_palantir():
    """Endpoint symulujący odpowiedź systemu Palantir"""
    if request.headers.get('Content-Type') != 'application/xml':
        logger.warning("Invalid content type received")
        return jsonify({"error": "Content-Type must be application/xml"}), 415

    try:
        xml_data = request.data.decode('utf-8')
        logger.info(f"Received Palantir alert:\n{xml_data}")

        try:
            root = ET.fromstring(xml_data)
        except ET.ParseError as e:
            logger.error(f"XML parsing error: {str(e)}")
            return jsonify({"error": "Malformed XML", "details": str(e)}), 400

        # Walidacja struktury XML
        if error := validate_xml_structure(root, ["ANALYTICAL_DATA"]):
            logger.warning(f"Validation error: {error}")
            return jsonify({"error": error}), 400

        # Generowanie ID alertu
        alert_id = f"PLTR-{datetime.utcnow().strftime('%Y%m%d%H%M%S')}"

        # Zapis do bazy
        fake_db.palantir_alerts.append({
            "id": alert_id,
            "timestamp": datetime.utcnow().isoformat(),
            "content": xml_data,
            "status": "RECEIVED"
        })

        logger.info(f"Alert {alert_id} registered successfully")

        # Formatowanie odpowiedzi
        response_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
        <PalantirResponse>
            <status>ACK</status>
            <alert_id>{alert_id}</alert_id>
            <timestamp>{datetime.utcnow().isoformat()}</timestamp>
        </PalantirResponse>"""

        return response_xml, 200, {'Content-Type': 'application/xml'}

    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500

@app.route('/sitaware/incidents', methods=['POST'])
def mock_sitaware():
    """Endpoint symulujący odpowiedź systemu SitaWare"""
    if request.headers.get('Content-Type') != 'application/xml':
        logger.warning("Invalid content type received")
        return jsonify({"error": "Content-Type must be application/xml"}), 415

    try:
        xml_data = request.data.decode('utf-8')
        logger.info(f"Received SitaWare incident:\n{xml_data}")

        try:
            root = ET.fromstring(xml_data)
        except ET.ParseError as e:
            logger.error(f"XML parsing error: {str(e)}")
            return jsonify({"error": "Malformed XML", "details": str(e)}), 400

        # Walidacja struktury XML
        if error := validate_xml_structure(root, ["RESOURCE_REQUIREMENTS"]):
            logger.warning(f"Validation error: {error}")
            return jsonify({"error": error}), 400

        # Generowanie ID incydentu
        incident_id = f"SW-{datetime.utcnow().strftime('%Y%m%d-%H%M%S')}"

        # Zapis do bazy
        fake_db.sitaware_incidents.append({
            "id": incident_id,
            "timestamp": datetime.utcnow().isoformat(),
            "content": xml_data,
            "status": "REGISTERED"
        })

        logger.info(f"Incedent {incident_id} registered successfully")

        # Formatowanie odpowiedzi
        response_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
        <SitaWareResponse>
            <IncidentID>{incident_id}</IncidentID>
            <Status>REGISTERED</Status>
            <Timestamp>{datetime.utcnow().isoformat()}</Timestamp>
        </SitaWareResponse>"""

        return response_xml, 200, {'Content-Type': 'application/xml'}

    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}", exc_info=True)
        return jsonify({"error": "Internal server error"}), 500

@app.route('/debug', methods=['GET'])
def debug():
    """Endpoint diagnostyczny zwracający stan symulatora"""
    return jsonify({
        "palantir_alerts": fake_db.palantir_alerts,
        "sitaware_incidents": fake_db.sitaware_incidents,
        "status": "operational",
        "timestamp": datetime.utcnow().isoformat()
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)

#!/usr/bin/env python3

import streamlit as st
import folium
from streamlit_folium import st_folium
import random
import time

st.set_page_config(layout="wide")
st.title("Hazard Risk Map in Subcarpathian Voivoidership")

# Region bounding box
lat_min, lat_max = 49.0, 50.5
lon_min, lon_max = 21.0, 23.5

# Sidebar controls
risk_threshold = st.sidebar.slider("Minimum Fire Risk to Display (%)", 0, 100, 0, 5)
if st.sidebar.button("🔁 Regenerate Fire Risk Points"):
    st.session_state.random_points = None
    st.session_state.display_index = 0

# Get fire risk color
def get_color(percent):
    if percent < 25:
        return 'green'
    elif percent < 60:
        return 'orange'
    else:
        return 'red'

# Generate random fire risk points
def generate_random_points(n):
    points = []
    for _ in range(n):
        lat = random.uniform(lat_min, lat_max)
        lon = random.uniform(lon_min, lon_max)
        risk = round(random.uniform(1, 100), 1)
        radius = random.uniform(4, 15)
        wind_angle = random.uniform(0, 360)
        points.append({
            "lat": lat,
            "lon": lon,
            "risk": risk,
            "radius": radius,
            "wind_angle": wind_angle
        })
    return points

# Initialize session state
if "random_points" not in st.session_state or st.session_state.random_points is None:
    st.session_state.random_points = generate_random_points(random.randint(5, 30))
if "display_index" not in st.session_state:
    st.session_state.display_index = 0

# Create folium map
m = folium.Map(location=[50.0413, 21.9990], zoom_start=8)

# Draw animated fire points up to current index
visible_points = st.session_state.random_points[:st.session_state.display_index + 1]

for i, point in enumerate(st.session_state["random_points"]):
    if point["risk"] < risk_threshold:
        continue

    color = get_color(point["risk"])
    radius = point.get("radius", 8)

    folium.Marker(
        location=[point["lat"], point["lon"]],
        icon=folium.DivIcon(
            html=f"""
                <div style="
                    position: relative;
                    animation: fadeIn 0.6s ease-out forwards;
                    animation-delay: {0.1 * i}s;
                    opacity: 0;
                    transform: translate(-50%, -50%);
                    text-align: center;
                    font-family: sans-serif;
                    line-height: 1.2;
                ">

                    <!-- Wind Arrow Container -->
                    <div style="
                        transform: rotate({point['wind_angle']}deg);
                        transform-origin: center;
                        margin-bottom: 2px;
                    ">
                        <div style="
                            display: inline-block;
                            font-size: 16px;
                            animation: wiggle 1.5s infinite ease-in-out;
                            transform-origin: center;
                        ">↑</div>
                    </div>

                    <!-- Circle Background -->
                    <div style="
                        width: {point['radius'] * 2}px;
                        height: {point['radius'] * 2}px;
                        background-color: {get_color(point['risk'])};
                        border-radius: 50%;
                        opacity: 0.6;
                        margin: 0 auto;
                        box-shadow: 0 0 8px rgba(0,0,0,0.3);
                        z-index: 0;
                    "></div>

                    <!-- Emoji + Percentage Label -->
                    <div style="
                        font-size: {2 + point['radius']}px;
                        font-weight: bold;
                        text-shadow: 1px 1px 2px white;
                        margin-top: -4px;
                        z-index: 2;
                    ">
                        🔥 {point['risk']}%
                    </div>
                </div>

                <style>
                @keyframes fadeIn {{
                    from {{ opacity: 0; transform: scale(0.8); }}
                    to {{ opacity: 1; transform: scale(1); }}
                }}
                @keyframes wiggle {{
                    0% {{ transform: rotate(0deg); }}
                    25% {{ transform: rotate(5deg); }}
                    50% {{ transform: rotate(0deg); }}
                    75% {{ transform: rotate(-5deg); }}
                    100% {{ transform: rotate(0deg); }}
                }}
                </style>
            """
        )
    ).add_to(m)

# Add legend
legend_html = """
<div style="
    position: fixed;
    bottom: 50px; left: 50px; width: 170px; height: 120px;
    background-color: white; z-index:9999;
    border:2px solid grey; padding: 10px; font-size: 14px;
    box-shadow: 2px 2px 6px rgba(0,0,0,0.3);
">
<b>🔥 Fire Risk Levels</b><br>
<i style="color:green;">●</i> Low (&lt;25%)<br>
<i style="color:orange;">●</i> Moderate (25–60%)<br>
<i style="color:red;">●</i> High (&gt;60%)<br>
</div>
"""
m.get_root().html.add_child(folium.Element(legend_html))

# Display map
st_folium(m, width=1000, height=650, key="animated_map")

if st.session_state.display_index + 1 < len(st.session_state.random_points):
    time.sleep(0.2)  # wait before showing the next point
    st.session_state.display_index += 1
    st.rerun()

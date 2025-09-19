# ============================================
# 1. IMPORTAR LIBRERÍAS NECESARIAS
# ============================================
import os
import osmnx as ox
import networkx as nx
import pandas as pd
import geopandas as gpd
import folium
import heapq
from math import radians, sin, cos, sqrt, asin

print("OSMnx y dependencias instaladas correctamente ✅")

# ============================================
# 2. CONFIGURACIÓN INICIAL DE OSMNX
# ============================================
ox.settings.use_cache = True
ox.settings.log_console = True

# ============================================
# 3. DESCARGAR Y GUARDAR LA RED VIAL DE MÉXICO
# ============================================
import os
import osmnx as ox

# Carpeta donde se guardará el archivo
ruta_base = "TT2"
os.makedirs(ruta_base, exist_ok=True)

# 👉 Cambia aquí el estado que quieras descargar
estado = "Veracruz"

try:
    print(f"Descargando red vial de {estado}... ⏳")
    G = ox.graph_from_place(estado, network_type="drive")

    # Guardar el archivo con nombre del estado
    archivo = os.path.join(ruta_base, f"{estado.replace(', ','_').replace(' ','_')}.graphml")
    ox.save_graphml(G, archivo)

    print(f"✅ Red vial de {estado} guardada en {archivo}")

except Exception as e:
    print(f"❌ Error al descargar {estado}: {e}")


# ============================================
# 4. CREAR DICCIONARIO DE COORDENADAS
# ============================================
node_coords = {node: (data['y'], data['x']) for node, data in G.nodes(data=True)}

# ============================================
# 5. CARGAR ONG Y VINCULAR A NODOS MÁS CERCANOS
# ============================================
df = pd.read_csv(os.path.join(ruta_base, "albergues_comedores_completo.csv"))  # columnas: nombre, latitud, longitud

for _, row in df.iterrows():
    nodo_ong = ox.distance.nearest_nodes(G, float(row["Longitud"]), float(row["Latitud"]))
    G.nodes[nodo_ong]["ong"] = row["Nombre"]

# ============================================
# 6. DEFINIR FUNCIONES AUXILIARES
# ============================================
def haversine(u, v, coords_dict):
    """Distancia ortodrómica en metros entre nodos u y v usando diccionario precalculado."""
    lat1, lon1 = coords_dict[u]
    lat2, lon2 = coords_dict[v]
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
    return 2 * 6371000 * asin(sqrt(a))  # metros

def astar(graph, start, goal, h, weight='length'):
    """Implementación personalizada de A*."""
    open_set = [(0, start)]
    came_from = {}
    gscore = {start: 0}
    
    while open_set:
        f, u = heapq.heappop(open_set)
        if u == goal:
            path = []
            while u in came_from:
                path.append(u)
                u = came_from[u]
            return [start] + path[::-1]
        for v, data in graph[u].items():
            w = data.get(weight, 1)
            tentative_g = gscore[u] + w
            if tentative_g < gscore.get(v, float('inf')):
                came_from[v] = u
                gscore[v] = tentative_g
                fscore = tentative_g + h(v, goal)
                heapq.heappush(open_set, (fscore, v))
    return None

# ============================================
# 7. DEFINIR COORDENADAS DE ORIGEN Y DESTINO
# ============================================
orig_point = (17.3912816,-93.9818479)   # guadalajara
dest_point = (17.3943646, -93.9961588)  # Puerto Vallarta

origin_node = ox.distance.nearest_nodes(G, orig_point[1], orig_point[0])
dest_node   = ox.distance.nearest_nodes(G, dest_point[1], dest_point[0])

print(f"Nodo de origen: {origin_node} {orig_point}")
print(f"Nodo de destino: {dest_node} {dest_point}")

# ============================================
# 8. EJECUTAR A* PERSONALIZADO
# ============================================
route_custom = astar(G, origin_node, dest_node, h=lambda u, v: haversine(u, v, node_coords))
print("\nRuta encontrada (personalizada):", route_custom)

# ============================================
# 9. EJECUTAR A* DE NETWORKX CON HEURÍSTICA
# ============================================
try:
    route_nx = nx.astar_path(
        G,
        origin_node,
        dest_node,
        heuristic=lambda u, v: haversine(u, v, node_coords),
        weight='length'
    )
    print(f"\nRuta encontrada con NetworkX ({len(route_nx)} nodos)")

    # Visualización
    fig, ax = ox.plot_graph_route(
        G,
        route_nx,
        route_linewidth=6,
        route_color='darkgreen',
        node_size=2,
        bgcolor='white'
    )
except nx.NetworkXNoPath:
    print("¡Error! No se encontró ruta entre los puntos.")
    fig, ax = ox.plot_graph(G, node_size=2)
    ax.scatter(orig_point[1], orig_point[0], c='red', s=100, zorder=3)
    ax.scatter(dest_point[1], dest_point[0], c='blue', s=100, zorder=3)

# ============================================
# 10. OBTENER COORDENADAS DE NODOS
# ============================================
origin_coords = node_coords[origin_node]
dest_coords = node_coords[dest_node]
# ============================================
# 11. VISUALIZAR RUTA EN MAPA INTERACTIVO (FOLIUM)
# ============================================

# Crear mapa centrado en el punto de origen
m = folium.Map(location=orig_point, zoom_start=6, tiles="cartodbpositron")

# Añadir marcador de origen y destino
folium.Marker(location=orig_point, popup="Origen (Guadalajara)", icon=folium.Icon(color="red")).add_to(m)
folium.Marker(location=dest_point, popup="Destino (Puerto Vallarta)", icon=folium.Icon(color="blue")).add_to(m)

# Extraer coordenadas de la ruta
route_coords = [(node_coords[node][0], node_coords[node][1]) for node in route_nx]

# Dibujar la ruta sobre el mapa
folium.PolyLine(route_coords, color="green", weight=4, opacity=0.8).add_to(m)

# Mostrar el mapa en notebook o guardarlo
m.save("ruta_interactiva.html")
print("✅ Mapa interactivo guardado como 'ruta_interactiva.html'")
# ============================================
# 12. AGREGAR ONG AL MAPA
# ============================================

for _, row in df.iterrows():
    folium.Marker(
        location=[row["Latitud"], row["Longitud"]],
        popup=row["Nombre"],
        icon=folium.Icon(color="purple", icon="home", prefix="fa")
    ).add_to(m)

# Guardar mapa final con ONG
m.save("ruta_con_ong.html")
print(" Mapa interactivo con ONG guardado como 'ruta_con_ong.html'")


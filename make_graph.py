import arcpy

arcpy.env.workspace = r"C:\Users\majak\Desktop\sem5\pag"
arcpy.env.overwriteOutput = True


# read shapefile where FID < 20
roads_shapefile = r"BDOT\kujawsko_pomorskie_m_Torun\L4_1_BDOT10k__OT_SKJZ_L.shp"
vertices_file = r"lab3/graf/vertices.txt"
edges_file = r"lab3/graf/edges.txt"

# Słownik do przechowywania unikalnych wierzchołków
vertices = {}
edges = []

# Funkcja dodająca wierzchołek (tworzy nowy, jeśli jeszcze nie istnieje)
def add_vertex(x, y, vertex_id):
    if (x, y) not in vertices:
        vertices[(x, y)] = vertex_id
    return vertices[(x, y)]

# Przetwarzanie shapefile z danymi dróg
with arcpy.da.SearchCursor(roads_shapefile, ["SHAPE@", "FID"]) as cursor:
    vertex_id = 0
    edge_id = 0

    for row in cursor:
        line = row[0]
        FID = row[1]
        
        # Pobieranie punktów początkowego i końcowego
        start_point = line.firstPoint
        end_point = line.lastPoint

        start_point = (round(start_point.X, 0), round(start_point.Y, 0))
        end_point = (round(end_point.X, 0), round(end_point.Y, 0))
        
        # Dodanie wierzchołków początkowego i końcowego
        start_vertex = add_vertex(start_point[0], start_point[1], vertex_id)
        vertex_id += 1 if start_vertex == vertex_id else 0  # Zwiększ ID jeśli wierzchołek jest nowy

        end_vertex = add_vertex(end_point[0], end_point[1], vertex_id)
        vertex_id += 1 if end_vertex == vertex_id else 0

        # Dodanie krawędzi do listy
        edges.append({
            "id": edge_id,
            "id_from": start_vertex,
            "id_to": end_vertex,
            "id_jezdni": FID,
            "dlugosc": line.length
        })
        edge_id += 1



# Zapis wierzchołków do pliku tekstowego
with open(vertices_file, "w") as vf:
    vf.write("id\tx\ty\n")
    for coords, v_id in vertices.items():
        vf.write(f"{v_id}\t{coords[0]}\t{coords[1]}\n")

# Zapis krawędzi do pliku tekstowego
with open(edges_file, "w") as ef:
    ef.write("id\tvertex_from\tvertex_to\tid_jezdni\tdlugosc\n")
    for edge in edges:
        ef.write(f"{edge['id']}\t{edge['id_from']}\t{edge['id_to']}\t{edge['id_jezdni']}\t{edge['dlugosc']}\n")

print("Liczba wierzchołków:", len(vertices))
print("Liczba krawędzi:", len(edges))
import arcpy
import heapq
import math
import os
from collections import defaultdict

class Toolbox:
    def __init__(self): 
        """Define the toolbox (the name of the toolbox is the name of the
        .pyt file)."""
        self.label = "Toolbox"
        self.alias = "toolbox"

        # List of tool classes associated with this toolbox
        self.tools = [Tool]

class Tool:
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "A* path finder"
        self.description = "Makes a graph from roads shapefile and counts the best path"

    def getParameterInfo(self):
        """Define the tool parameters."""

        # Input road features
        SKJZ = arcpy.Parameter(
            displayName="Road features file",
            name="SKJZ",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")

        # Input start and end points
        start_end = arcpy.Parameter(
            displayName="Start and end points",
            name="points",
            datatype="GPFeatureRecordSetLayer", # GPFeatureRecordSetLayer allows user to create a point layer on the spot
            parameterType="Required",
            direction="Input")
        # start_end.value = "points.lyrx" - layer definition file
        
        # Output Folder for creating graph files
        output_folder = arcpy.Parameter(
            displayName="Output Folder",
            name="folder",
            datatype="DEFolder",
            parameterType="Required",
            direction="Input")

        # 2 options: shortest path, fastest path
        algorithm_type = arcpy.Parameter(
            displayName="Type of path",
            name="algorithm",
            datatype="GPString",
            parameterType="Required",
            direction="Input")

        algorithm_type.filter.type = "ValueList"
        algorithm_type.filter.list = ["Shortest Path", "Fastest Path"]

        
        # derived outputs to display the final route
        output_visited_vertices = arcpy.Parameter(
            displayName="Visited vertices",
            name="vis_vertices",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        output_shortest_path = arcpy.Parameter(
            displayName="Shortest Path",
            name="shortest_path",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        output_fastest_path = arcpy.Parameter(
            displayName="Fastest Path",
            name="fastest_path",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        params = [SKJZ, start_end, output_folder, algorithm_type, output_visited_vertices, output_shortest_path, output_fastest_path]
        return params

    def isLicensed(self):
        """Set whether the tool is licensed to execute."""
        return True

    def updateParameters(self, parameters):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""
        return

    def updateMessages(self, parameters):
        """Modify the messages created by internal validation for each tool
        parameter. This method is called after internal validation."""

        return

    def execute(self, parameters, messages):
        """The source code of the tool.""" 

        arcpy.env.overwriteOutput = True

        # User inputs
        SKJZ = parameters[0].valueAsText
        start_end_layer = parameters[1].valueAsText
        folder = parameters[2].valueAsText
        path_type = parameters[3].valueAsText

        # Output files
        OUT_vertices_txt = folder + r"\vertices.txt"
        OUT_edges_txt = folder + r"\edges.txt"
        OUT_vertices_shp = folder + r"\vertices"
        OUT_visited_vertices_txt = folder + r"\visited_vertices.txt"
        OUT_path = folder + r"\path.txt"
        OUT_visited_edges_txt = folder + r"\visited_roads.txt"
        OUT_visited_vertices  = folder + r"\visited_vertices"
        OUT_shortest_path = folder + r"\shortest_path"
        OUT_fastest_path = folder + r"\fastest_path"

        # Check if stard_end_layer is a point layer
        desc = arcpy.Describe(start_end_layer)
        if desc.shapeType != "Point":
            arcpy.AddError("Start and end points must be points")
            return
        
        # Check if SKJZ is a line layer
        desc = arcpy.Describe(SKJZ)
        if desc.shapeType != "Polyline":
            print(desc.shapeType)
            arcpy.AddError("Road features must be lines")
            return

        # check if the output folder exists
        if not os.path.exists(folder):
            arcpy.AddError("Output folder does not exist")
            return
        

        vertices = {}
        edges = []

        # Create vertex if one doesn't exist with the same coords
        def add_vertex(x, y, vertex_id):
            if (x, y) not in vertices:
                vertices[(x, y)] = {'id': vertex_id, 'x': x, 'y': y, 'edges': []}
            return vertices[(x, y)]['id']

        # Speed by road class
        def speed(klasa):
            kmh2ms = 1000 / 3600 
            if klasa == 'A': # autostrada
                return 140 * kmh2ms
            elif klasa == 'S': # droga ekspresowa
                return 120 * kmh2ms
            elif klasa == 'GP': # główna ruchu
                return 70 * kmh2ms
            elif klasa == 'G': # główna
                return 60 * kmh2ms
            elif klasa == 'Z': # zbiorcza
                return 50 * kmh2ms
            elif klasa == 'L': # lokalna
                return 40 * kmh2ms
            elif klasa == 'D': # dojazdowa
                return 20 * kmh2ms
            elif klasa == 'I': # inna
                return 20 * kmh2ms
            else:
                return 10
        
        arcpy.AddMessage("Creating graph...")

        # Read road file 
        with arcpy.da.SearchCursor(SKJZ, ["SHAPE@", "FID", "klasaDrogi", "kierunek"]) as cursor:
            vertex_id = 0
            edge_id = 0

            # iterate through each feature
            for row in cursor:
                line = row[0]
                FID = row[1]
                klasa = row[2]
                kierunek = row[3]
                speedms = speed(klasa)

                # Read start and end point, round to one meter
                start_point = line.firstPoint
                end_point = line.lastPoint

                start_point = (round(start_point.X, 0), round(start_point.Y, 0))
                end_point = (round(end_point.X, 0), round(end_point.Y, 0))
                
                # Add vertices
                start_vertex = add_vertex(start_point[0], start_point[1], vertex_id)
                vertex_id += 1 if start_vertex == vertex_id else 0

                end_vertex = add_vertex(end_point[0], end_point[1], vertex_id)
                vertex_id += 1 if end_vertex == vertex_id else 0

                edges.append({
                    "id": edge_id,
                    "from": start_vertex,
                    "to": end_vertex,
                    "road_id": FID,
                    "length": line.length,
                    "speed": speedms,
                    'direction': kierunek
                })
                edge_id += 1
                
                # add edges to the vertex file
                vertices[start_point]['edges'].append(edge_id)
                vertices[end_point]['edges'].append(edge_id)

        arcpy.AddMessage("Graph created, writing to files...")

        # Write vertices
        with open(OUT_vertices_txt, "w") as vf:
            vf.write("id\tx\ty\tedges\n")
            for v in vertices.values():
                vf.write(f"{v['id']}\t{v['x']}\t{v['y']}\t{','.join(map(str, v['edges']))}\n")

        # Write edges
        with open(OUT_edges_txt, "w") as ef:
            ef.write("id\tfrom\tto\troad_id\tlength\tspeed\tdirection\n")
            for edge in edges:
                ef.write(f"{edge['id']}\t{edge['from']}\t{edge['to']}\t{edge['road_id']}\t{edge['length']}\t{edge['speed']}\t{edge['direction']}\n")

        arcpy.AddMessage(f"Vertex count: {len(vertices)}")
        arcpy.AddMessage(f"Edge count: {len(edges)} \n")

        # Makes points from the vertex.txt file
        arcpy.management.XYTableToPoint(
            in_table=OUT_vertices_txt,
            out_feature_class=OUT_vertices_shp,
            x_field="x",
            y_field="y",
            z_field=None,
            coordinate_system='PROJCS["ETRS_1989_UWPP_1992",GEOGCS["GCS_ETRS_1989",DATUM["D_ETRS_1989",SPHEROID["GRS_1980",6378137.0,298.257222101]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]],PROJECTION["Gauss_Kruger"],PARAMETER["False_Easting",500000.0],PARAMETER["False_Northing",-5300000.0],PARAMETER["Central_Meridian",19.0],PARAMETER["Scale_Factor",0.9993],PARAMETER["Latitude_Of_Origin",0.0],UNIT["Meter",1.0]];-5119200 -15295100 10000;-100000 10000;-100000 10000;0.001;0.001;0.001;IsHighPrecision'
        )

        xs = []
        ys = []

        # Get the coordinates of the start and end points
        with arcpy.da.SearchCursor(start_end_layer, ["SHAPE@XY"]) as cursor:
            for row in cursor:
                point = row[0]
                x = point[0]
                y = point[1]
                xs.append(x)
                ys.append(y)
            if len(xs) != 2:
                arcpy.AddError("Two points required")
                return

        start_point, end_point = (xs[0], ys[0]), (xs[1], ys[1])

        # Find nearest vertex to the given points
        arcpy.analysis.Near(
            in_features=start_end_layer,
            near_features=OUT_vertices_shp,
            search_radius=None,
            location="NO_LOCATION",
            angle="NO_ANGLE",
            method="PLANAR",
            field_names="NEAR_FID NEAR_FID;NEAR_DIST NEAR_DIST",
            distance_unit=""
        )

        near_fids = []
        with arcpy.da.SearchCursor(start_end_layer, ["SHAPE@", "NEAR_FID"]) as cursor:
            for row in cursor:
                point = row[0]
                near_fids.append(row[1])

        start = near_fids[0]
        end = near_fids[1]

        # Read graph from the files and find the path
        def read_graph_directed(filename):
            f = open(filename)
            g = defaultdict(list)
            road_ids = {}
            road_lengths = {}
            road_speeds = {}
            
            f.readline()
            for line in f:
                e = line.split()
                from_vertex, to_vertex, road_id, direction = int(e[1]), int(e[2]), int(e[3]), int(e[6])
                length, speed = float(e[4]), float(e[5])

                # Add edge in one direction only
                if direction == 0: # two-way road
                    # store neighbors and length
                    g[from_vertex].append((to_vertex, length, speed))
                    g[to_vertex].append((from_vertex, length, speed))

                elif direction == 1: # one-way road (start -> end)
                    g[from_vertex].append((to_vertex, length, speed))

                elif direction == 2: # one-way road (end -> start)
                    g[to_vertex].append((from_vertex, length, speed))

                 # store road ID and length in both directions
                road_ids[(from_vertex, to_vertex)] = road_id
                road_ids[(to_vertex, from_vertex)] = road_id
                road_lengths[(from_vertex, to_vertex)] = length
                road_lengths[(to_vertex, from_vertex)] = length
                road_speeds[(from_vertex, to_vertex)] = speed
                road_speeds[(to_vertex, from_vertex)] = speed

            # else if direction == 3, do nothing (road is not used)

            f.close()
                
            return g, road_ids, road_lengths, road_speeds

        # Retrieve path from the previous nodes
        def retrieve_path(prev, a, b, road_ids):
            path = [b]
            road_path = []
            while b != a:
                b = prev[b]
                path.append(b)
                road_path.append(road_ids[(b, path[-2])])
            path.reverse()
            road_path.reverse()
            return path, road_path

        # Estimate euclidean distance from one node to another
        def heuristic_shortest(v, goal, vertices):
            vx, vy = vertices[v]['x'], vertices[v]['y']
            gx, gy = vertices[goal]['x'], vertices[goal]['y']
            return math.sqrt((vx - gx) ** 2 + (vy - gy) ** 2)

        # A* pathfinding algorithm with logging
        def a_star_path_shortest(graph, road_ids, vertices, start, end):
            open_set = []  # priority queue
            heapq.heappush(open_set, (0, start))  # (f-score, vertex)
            g_score = {start: 0} # known cost to reach each node
            prev = {}

            # Logging variables
            S_size = 0
            visited_nodes = []

            while open_set:
                # get the node with best f-score (lowest cost)
                _, current = heapq.heappop(open_set)
                visited_nodes.append(current)

                if current == end:
                    arcpy.AddMessage(f"Number of vertices in S set: {S_size}")
                    arcpy.AddMessage(f"Total number of visited vertices: {len(visited_nodes)}")
                    
                    return retrieve_path(prev, start, end, road_ids)

                for neighbor, length, _ in graph[current]:
                    if neighbor not in g_score:
                        g_score[neighbor] = float('inf')
                    # calculate tentative cost to reach neighbor (cost of reaching current + distance to neighbor)
                    tentative_g_score = g_score[current] + length
                    if tentative_g_score < g_score[neighbor]:
                        prev[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        # calculate f score:  total cost to reach neigbor + heuristic estimate
                        f_score = g_score[neighbor] + heuristic_shortest(neighbor, end, vertices)
                        heapq.heappush(open_set, (f_score, neighbor))

                # update the size of the set S (open_set)
                S_size = len(open_set)

            return None, None
        
        def heuristic_fastest(v, goal, vertices, road_speeds):
            vx, vy = vertices[v]['x'], vertices[v]['y']
            gx, gy = vertices[goal]['x'], vertices[goal]['y']
            distance = math.sqrt((vx - gx) ** 2 + (vy - gy) ** 2)
            max_speed = max(road_speeds.values())

            return distance / max_speed

        def a_star_path_fastest(graph, road_ids, road_speeds, vertices, start, end):
            open_set = []  
            heapq.heappush(open_set, (0, start)) 
            g_score = {start: 0}
            prev = {}

            S_size = 0
            visited_nodes = []

            while open_set:
                _, current = heapq.heappop(open_set)
                visited_nodes.append(current)

                if current == end:
                    arcpy.AddMessage(f"Number of vertices in S set: {S_size}")
                    arcpy.AddMessage(f"Total number of visited vertices: {len(visited_nodes)}")

                    return retrieve_path(prev, start, end, road_ids)

                for neighbor, length, speed in graph[current]:
                    if neighbor not in g_score:
                        g_score[neighbor] = float('inf')
                    tentative_g_score = g_score[current] + (length / speed) 
                    if tentative_g_score < g_score[neighbor]:
                        prev[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = g_score[neighbor] + heuristic_fastest(neighbor, end, vertices, road_speeds) 
                        heapq.heappush(open_set, (f_score, neighbor))

                S_size = len(open_set)

            return None, None

        g, road_ids, road_lengths, road_speeds = read_graph_directed(OUT_edges_txt)


        vertices_dict = {}
        with open(OUT_vertices_txt, 'r') as vf:
            vf.readline()
            for line in vf:
                parts = line.split()
                v_id = int(parts[0])
                x, y = float(parts[1]), float(parts[2])
                vertices_dict[v_id] = {'x': x, 'y': y}

        arcpy.AddMessage("\nFinding path...")

        # Get path and roads
        if path_type == "Shortest Path":
            path, road_path = a_star_path_shortest(g, road_ids, vertices_dict, start, end)
        elif path_type == "Fastest Path":
            path, road_path = a_star_path_fastest(g, road_ids, road_speeds, vertices_dict, start, end)

        if path is None:
            arcpy.AddMessage("No path between given vertices")
            return
        else:
            arcpy.AddMessage(f"Path found. Writing to file path.txt")

        # Write the path to a text file
        with open(OUT_path, 'w') as f:
            for i in path:
                f.write(str(i) + '\n')

        # find the vertices in the path and write the coords to the file
        with open(OUT_vertices_txt, 'r') as f:
            vertices = f.readlines()
            vertices = [x.strip() for x in vertices]
            path_vertex = [vertices[i+2].split()[0] for i in path]

        # write the vertices in path to a file
        with open(OUT_visited_vertices_txt, 'w') as f:
            f.write("id\tx\ty\tedges\n")
            for i in path_vertex:
                f.write(vertices[int(i)] + '\n')

        # write the road path to a file
        with open(OUT_visited_edges_txt, 'w') as f:
            for i in road_path:
                f.write(str(i) + '\n')

        # Seconds to hours, minutes, seconds
        def convert_time(total_seconds):
            hours, minutes, seconds = 0, 0, 0
            hours = total_seconds // 3600
            minutes = (total_seconds % 3600) // 60
            seconds = total_seconds % 60
            if hours != 0:
                return f"{int(hours)}h {int(minutes)}min {int(seconds)}sec"
            elif minutes != 0:
                return f"{int(minutes)}min {int(seconds)}sec"
            else:
                return f"{int(seconds)}sec"
        
        # Meters to kilometers and meters
        def convert_len(length):
            km = 0
            if length > 1000:
                km = length // 1000
            m = length % 1000
            
            if km != 0:
                return f"{int(km)}km {round(m)}m"
            else:
                return f"{round(m)} m"

        # Count total length and time of the path
        def len_time(path):
            total_length = 0
            total_time = 0

            for i in range(len(path) - 1):
                from_vertex = path[i]
                to_vertex = path[i + 1]
                
                segment_length = road_lengths[(from_vertex, to_vertex)]
                segment_speed = road_speeds[(from_vertex, to_vertex)]
                
                total_length += segment_length
                total_time += segment_length / segment_speed
            
            return total_length, total_time

        total_length, total_time = len_time(path)

                
        arcpy.AddMessage(f"\n--------------- Route ---------------")
        arcpy.AddMessage(f"Total length of the route: {convert_len(total_length)}")
        arcpy.AddMessage(f"Total time of the route: {convert_time(total_time)}")

        # Save and add visited vertices to display
        arcpy.management.XYTableToPoint(
            in_table=OUT_visited_vertices_txt,
            out_feature_class=OUT_visited_vertices,
            x_field="x",
            y_field="y",
            z_field=None,
            coordinate_system='PROJCS["ETRS_1989_UWPP_1992",GEOGCS["GCS_ETRS_1989",DATUM["D_ETRS_1989",SPHEROID["GRS_1980",6378137.0,298.257222101]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]],PROJECTION["Gauss_Kruger"],PARAMETER["False_Easting",500000.0],PARAMETER["False_Northing",-5300000.0],PARAMETER["Central_Meridian",19.0],PARAMETER["Scale_Factor",0.9993],PARAMETER["Latitude_Of_Origin",0.0],UNIT["Meter",1.0]];-5119200 -15295100 10000;-100000 10000;-100000 10000;0.001;0.001;0.001;IsHighPrecision'
        )
        # vertices_dispname = os.path.splitext("visisted_vertices")[0]
        # arcpy.MakeFeatureLayer_management(OUT_visited_vertices, vertices_dispname)
        # arcpy.SetParameterAsText(3, vertices_dispname)
        # arcpy.AddMessage("Added visited vertices to display.")

        # Create an SQL expression for selecting only visited roads
        def create_sql_expression(road_path):
            sql_expression = "FID = " + str(road_path[0])
            for i in road_path[1:]:
                sql_expression += " Or FID = " + str(i)

            return sql_expression

        sql_expression = create_sql_expression(road_path)

        # Select visited roads to a new file
        arcpy.analysis.Select(  
            in_features=SKJZ,
            out_feature_class=OUT_shortest_path,
            where_clause=sql_expression
        )

        # Display the route
        if path_type == "Shortest Path":
            arcpy.analysis.Select(  
                in_features=SKJZ,
                out_feature_class=OUT_shortest_path,
                where_clause=sql_expression
            )
            roads_dispname = os.path.splitext("Shortest Path")[0]
            arcpy.MakeFeatureLayer_management(OUT_shortest_path, roads_dispname)
            arcpy.SetParameterAsText(4, roads_dispname)


        elif path_type == "Fastest Path":
            arcpy.analysis.Select(  
                in_features=SKJZ,
                out_feature_class=OUT_fastest_path,
                where_clause=sql_expression
            )
            roads_dispname = os.path.splitext("Fastest Path")[0]
            arcpy.MakeFeatureLayer_management(OUT_fastest_path, roads_dispname)
            arcpy.SetParameterAsText(5, roads_dispname)

        arcpy.AddMessage("\nPath added to display.")

        return

    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""

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

        SKJZ = arcpy.Parameter(
            displayName="Road shapefile",
            name="SKJZ",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")

        #GPFeatureRecordSetLayer
        start_end = arcpy.Parameter(
            displayName="Start and end point",
            name="points",
            datatype="GPFeatureRecordSetLayer",
            parameterType="Required",
            direction="Input")
        # start_end.value = r"D:\sem5\pag\lab3-2\points.lyrx"
        
        output_folder = arcpy.Parameter(
            displayName="Output Folder",
            name="folder",
            datatype="DEFolder",
            parameterType="Required",
            direction="Input")

        algorithm_type = arcpy.Parameter(
            displayName="Type of path",
            name="algorithm",
            #2 options: shortest path, fastest path
            datatype="GPString",
            parameterType="Required",
            direction="Input")

        algorithm_type.filter.type = "ValueList"
        algorithm_type.filter.list = ["Shortest Path", "Fastest Path"]

        output_visited_vertices = arcpy.Parameter(
            displayName="Visited vertices",
            name="vis_vertices",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        output_visited_roads = arcpy.Parameter(
            displayName="Visited Roads",
            name="vis_roads",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        params = [SKJZ, start_end, output_folder, algorithm_type, output_visited_vertices, output_visited_roads]
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


        SKJZ = parameters[0].valueAsText
        start_end_layer = parameters[1].valueAsText
        folder = parameters[2].valueAsText
        path_type = parameters[3].valueAsText

        OUT_vertices_txt = folder + r"\vertices.txt"
        OUT_edges_txt = folder + r"\edges.txt"
        OUT_vertices_shp = folder + r"\vertices"
        OUT_visited_vertices_txt = folder + r"\visited_vertices.txt"
        OUT_visited_edges_txt = folder + r"\visited_roads.txt"
        OUT_visited_vertices  = folder + r"\visited_vertices"
        OUT_visited_roads = folder + r"\visited_roads"

        vertices = {}
        edges = []

        # Create vertex if one doesn't exist with the same coords
        def add_vertex(x, y, vertex_id):
            if (x, y) not in vertices:
                vertices[(x, y)] = {'id': vertex_id, 'x': x, 'y': y, 'edges': []}
            return vertices[(x, y)]['id']

        
        def speed(klasa):
            kmh2ms = 1000 / 3600
            if klasa == 'A':
                return 140 * kmh2ms
            elif klasa == 'S':
                return 120 * kmh2ms
            elif klasa == 'GP':
                return 70 * kmh2ms
            elif klasa == 'G':
                return 60 * kmh2ms
            elif klasa == 'Z':
                return 50 * kmh2ms
            elif klasa == 'L':
                return 40 * kmh2ms
            elif klasa == 'D':
                return 20 * kmh2ms
            elif klasa == 'I':
                return 20 * kmh2ms
            else:
                return 10
        
        arcpy.AddMessage("Creating graph...")

        # Read road shp
        with arcpy.da.SearchCursor(SKJZ, ["SHAPE@", "FID", "klasaDrogi"]) as cursor:
            vertex_id = 0
            edge_id = 0

            for row in cursor:
                line = row[0]
                FID = row[1]
                klasa = row[2]
                speedms = speed(klasa)

                
                # read start and end point, round to one meter
                start_point = line.firstPoint
                end_point = line.lastPoint

                start_point = (round(start_point.X, 0), round(start_point.Y, 0))
                end_point = (round(end_point.X, 0), round(end_point.Y, 0))
                
                start_vertex = add_vertex(start_point[0], start_point[1], vertex_id)
                vertex_id += 1 if start_vertex == vertex_id else 0

                end_vertex = add_vertex(end_point[0], end_point[1], vertex_id)
                vertex_id += 1 if end_vertex == vertex_id else 0

                # add edges
                edges.append({
                    "id": edge_id,
                    "from": start_vertex,
                    "to": end_vertex,
                    "road_id": FID,
                    "length": line.length,
                    "speed": speedms
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
            ef.write("id\tfrom\tto\troad_id\tlength\tspeed\n")
            for edge in edges:
                ef.write(f"{edge['id']}\t{edge['from']}\t{edge['to']}\t{edge['road_id']}\t{edge['length']}\t{edge['speed']}\n")

        arcpy.AddMessage(f"Vertex count: {len(vertices)}")
        arcpy.AddMessage(f"Edge count: {len(edges)}")

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

        # find nearest vertex to the given points
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

        # this is correct as long as OBJECTID is the same as id assigned in the vertices.txt file
        start = near_fids[0]
        end= near_fids[1]

        # start, end = 566, 7063

        def read_graph_undirected(filename):
            f = open(filename)
            g = defaultdict(list)
            road_ids = {}
            road_lengths = {}
            road_speeds = {}
            
            f.readline()
            for line in f:
                e = line.split()
                from_vertex, to_vertex, road_id = int(e[1]), int(e[2]), int(e[3])
                length, speed = float(e[4]), float(e[5])

                # store neighbors and length for each direction
                g[from_vertex].append((to_vertex, length, speed))
                g[to_vertex].append((from_vertex, length, speed))

                # store road ID and length for each direction (undirected graph)
                road_ids[(from_vertex, to_vertex)] = road_id
                road_ids[(to_vertex, from_vertex)] = road_id
                road_lengths[(from_vertex, to_vertex)] = length
                road_lengths[(to_vertex, from_vertex)] = length
                road_speeds[(from_vertex, to_vertex)] = speed
                road_speeds[(to_vertex, from_vertex)] = speed
                
            return g, road_ids, road_lengths, road_speeds

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

        # estimate euclidean distance from one node to another
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

            return None

        g, road_ids, road_lengths, road_speeds = read_graph_undirected(OUT_edges_txt)

        vertices_dict = {}
        with open(OUT_vertices_txt, 'r') as vf:
            vf.readline()
            for line in vf:
                parts = line.split()
                v_id = int(parts[0])
                x, y = float(parts[1]), float(parts[2])
                vertices_dict[v_id] = {'x': x, 'y': y}

        arcpy.AddMessage("Finding path...")

        if path_type == "Shortest Path":
            path, road_path = a_star_path_shortest(g, road_ids, vertices_dict, start, end)
        elif path_type == "Fastest Path":
            path, road_path = a_star_path_fastest(g, road_ids, road_speeds, vertices_dict, start, end)

        if path is None:
            arcpy.AddMessage("No path between given vertices")
            return
        else:
            arcpy.AddMessage(f"Path found: {path}")

        # count total length and time
        def convert_time(total_seconds):
            hours = total_seconds // 3600
            minutes = (total_seconds % 3600) // 60
            seconds = total_seconds % 60
            return f"{int(hours)}h {int(minutes)}min {int(seconds)}sec"

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
        formatted_time = convert_time(total_time)

        arcpy.AddMessage(f"Total length of route: {round(total_length, 2)} meters")
        arcpy.AddMessage(f"Total time: {formatted_time}")
        arcpy.AddMessage(f"Writing path to files...")
        # find the vertices in the path and write the coords to the file
        with open(OUT_vertices_txt, 'r') as f:
            vertices = f.readlines()
            vertices = [x.strip() for x in vertices]
            path = [vertices[i+2].split()[0] for i in path]

        # write the path to a file
        with open(OUT_visited_vertices_txt, 'w') as f:
            f.write("id\tx\ty\tedges\n")
            for i in path:
                f.write(vertices[int(i)] + '\n')

        # write the road path to a file
        with open(OUT_visited_edges_txt, 'w') as f:
            f.write("id\tx\ty\tedges\n")
            for i in road_path:
                f.write(str(i) + '\n')

        # Add visited vertices to display
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

        # Add visited roads to display
        def create_sql_expression(road_path):
            sql_expression = "FID = " + str(road_path[0])
            for i in road_path[1:]:
                sql_expression += " Or FID = " + str(i)

            return sql_expression

        sql_expression = create_sql_expression(road_path)

        arcpy.analysis.Select(  
            in_features=SKJZ,
            out_feature_class=OUT_visited_roads,
            where_clause=sql_expression
        )

        if path_type == "Shortest Path":
            roads_dispname = os.path.splitext("Shortest Path")[0]
        elif path_type == "Fastest Path":
            roads_dispname = os.path.splitext("Fastest Path")[0]

        arcpy.MakeFeatureLayer_management(OUT_visited_roads, roads_dispname)
        arcpy.SetParameterAsText(4, roads_dispname)
        arcpy.AddMessage("Added visited roads to display.")

        return

    def postExecute(self, parameters):
        """This method takes place after outputs are processed and
        added to the display."""


# Course: CS261 - Data Structures
# Author: Jack Gallivan
# Assignment: 6
# Description: Undirected graph ADT implemented using python's list and dictionary.

class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Add new vertex to the graph

        :param v: Name of vertex to add.
        """
        if v not in self.adj_list:
            self.adj_list[v] = []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        Add edge to the graph

        :param u: Name of first vertex.
        :param v: Name of second vertex.
        """
        # Do nothing if vertices have same name.
        if u == v:
            return
        # Add the vertices if they don't exist.
        self.add_vertex(u)
        self.add_vertex(v)
        # If the vertices aren't connected already, connect them.
        if v not in self.adj_list[u]:
            self.adj_list[u].append(v)
            self.adj_list[v].append(u)

    def remove_edge(self, u: str, v: str) -> None:
        """
        Remove edge from the graph

        :param u: Name of first vertex.
        :param v: Name of second vertex.
        """
        # Do nothing if either vertex is not in the list.
        if u not in self.adj_list or v not in self.adj_list:
            return
        # Remove the vertices from each other's list, if they are in there at all.
        if v in self.adj_list[u]:
            self.adj_list[u].remove(v)
            self.adj_list[v].remove(u)

    def remove_vertex(self, v: str) -> None:
        """
        Remove vertex and all connected edges

        :param v: Name of vertex to remove.
        """
        # Do nothing if the vertex does not exist.
        if v not in self.adj_list:
            return
        # Remove the vertex from the list of each vertex it is connected to, to remove its edges.
        for element in self.adj_list[v]:
            self.adj_list[element].remove(v)
        # Remove the vertex itself, which removes remaining directed edges.
        self.adj_list.pop(v)

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)

        :return: List of every vertex in the graph.
        """
        vertex_list = []
        for key in self.adj_list:
            vertex_list.append(key)
        return vertex_list

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)

        :return: List of every edge in the graph, as unordered tuples of vertices.
        """
        edge_list = []
        # Iterate through every vertex, call the vertices v1.
        for v1 in self.adj_list:
            # Iterate through v1's connections, call the vertices v2.
            for v2 in self.adj_list[v1]:
                # Check if the edge has already been added (v2 was iterated through in the outer loop).
                if (v2, v1) in edge_list:
                    continue
                # Add the edge to the list.
                edge_list.append((v1, v2))
        return edge_list

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise

        :param path: A list of vertices to visit in order.
        :return: True if the path is connected. False otherwise.
        """
        # Check whether vertex path[0] exists in the graph.
        if len(path) > 0 and path[0] not in self.adj_list:
            return False
        # Check connections between each consecutive vertex.
        for i in range(len(path) - 1):
            # If no edge between consecutive vertices, return False:
            if path[i+1] not in self.adj_list[path[i]]:
                # Note: If path[i+1] does not exist, this is implicitly true.
                return False
        # Valid path.
        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order

        :param v_start: Name of vertex to start search at.
        :param v_end: Name of vertex to end search at. If unspecified, all vertices connected to v_start will be
            visited.
        :return: List of vertices visited, in order visited. If v_end is specified, the search is stopped once it is
            visited.
        """
        # Return empty list if v_start does not exist.
        if v_start not in self.adj_list:
            return []
        # Initialize stack and visited list.
        stack = [v_start]
        visited = []
        # Do until stack empty:
        while len(stack) > 0:
            v = stack.pop(-1)  # v = vertex popped from top of stack.
            # If v has not yet been visited:
            if v not in visited:
                visited.append(v)
                if v_end is not None and v == v_end:
                    break  # If v == v_end: we're done.
                adj = self.adj_list[v]
                adj.sort(reverse=True)
                # For every vertex connected to v, in descending order:
                for element in adj:
                    # If the connected vertex has not been visited, add it to the stack:
                    if element not in visited:
                        stack.append(element)
        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order

        :param v_start: Name of starting vertex
        :param v_end: Name of vertex to end search at. If unspecified, all vertices connected to v_start will be
            visited.
        :return: List of vertices visited, in order visited. If v_end is specified, the search is stopped once it is
            visited.
        """
        # Return empty list if v_start does not exist.
        if v_start not in self.adj_list:
            return []
        # Initialize queue and visited list.
        queue = [v_start]
        visited = []
        # Do until queue is empty:
        while len(queue) > 0:
            v = queue.pop(0)  # v = vertex dequeued from beginning of queue.
            # If v has not yet been visited:
            if v not in visited:
                visited.append(v)
                if v_end is not None and v == v_end:
                    break  # If v == v_end: we're done.
                adj = self.adj_list[v]
                adj.sort()
                # For every vertex connected to v, in ascending order:
                for element in adj:
                    # If the connected vertex has not been visited and is not in the queue, add it to the queue:
                    if element not in visited and element not in queue:
                        queue.append(element)
        return visited

    def count_connected_components(self) -> int:
        """
        Return number of connected components in the graph

        :return: Number of connected components in the graph.
        """
        # Initialize components to 0 and list of visited vertices to empty.
        components = 0
        visited = []
        # Iterate through every vertex in the graph:
        for v in self.adj_list:
            if v in visited:
                continue  # If v has already been visited, continue loop.
            # Get every vertex connected to v and add it to the list of visited vertices. +1 components visited.
            visited.extend(self.bfs(v))
            components += 1
        return components

    def has_cycle(self) -> bool:
        """
        Return True if graph contains a cycle, False otherwise

        :return: True if there is a cycle in the graph. False otherwise.
        """
        # Get the list of all edges in the graph.
        edge_list_original = self.get_edges()
        # Get one arbitrary vertex from each graph component to start our cycle searches at.
        vertex_list = []
        visited = []
        for v in self.adj_list:
            if v in visited:
                continue
            vertex_list.append(v)
            visited.extend(self.bfs(v))
        # Check for a cycle starting at an arbitrary vertex in each component:
        for v_start in vertex_list:
            edge_list = edge_list_original.copy()  # Copy the list of all edges
            # Initialize list of visited vertices and queue of connected vertices to the vertices adjacent to v_start.
            visited = []
            queue = []
            queue.extend(self.adj_list[v_start])
            visited.extend(queue)
            # Remove every edge containing v_start from edge_list.
            edge_list[:] = [edge for edge in edge_list if not (v_start in edge)]
            # Iterate through the queue until empty:
            while len(queue) > 0:
                v = queue.pop(0)  # v = vertex dequeued from beginning of queue.
                # Get every edge containing v that remains in edge_list.
                edges = [edge for edge in edge_list if v in edge]
                # Remove those edges from edge_list.
                edge_list[:] = [edge for edge in edge_list if not (v in edge)]
                for edge in edges:
                    v_next = edge[1-edge.index(v)]  # v_next = a vertex adjacent to v.
                    if v_next in visited:
                        # If v_next has already been visited, the graph contains a cycle.
                        return True
                    # Add v_next to visited and the queue.
                    visited.append(v_next)
                    queue.append(v_next)
        # No cycles were found:
        return False


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())

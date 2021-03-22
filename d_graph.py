# Course: CS261 - Data Structures
# Author: Jack Gallivan
# Assignment: 6
# Description: Directed graph ADT implemented using python's list and dictionary.

import heapq


class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Add new vertex to the graph

        :return: Number of vertices in the graph after adding a vertex.
        """
        # Add a new row of zeros to the matrix:
        self.adj_matrix.append([0] * self.v_count)
        # Append 0 to every row in the matrix to account for the added vertex:
        for row in self.adj_matrix:
            row.append(0)
        self.v_count += 1
        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Add edge to the graph

        :param src: Index of the head vertex of the edge being added.
        :param dst: Index of the tail vertex of the edge being added.
        :param weight: The weight of the edge being added/updated.
        """
        # Do nothing if either vertex doesn't exist, weight < 1, or src = dst.
        if not 0 <= src < self.v_count or not 0 <= dst < self.v_count or weight < 1 or src == dst:
            return
        # Add/update the edge's weight by setting the value in the adjacency matrix.
        self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Remove edge from the graph

        :param src: Index of the head vertex of the edge being removed.
        :param dst: Index of the tail vertex of the edge being removed.
        """
        # Do nothing if either vertex or the edge does not exist.
        if not 0 <= src < self.v_count or not 0 <= dst < self.v_count or self.adj_matrix[src][dst] == 0:
            return
        # Remove the edge by setting the edge value in the adjacency matrix to zero.
        self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)

        :return: A list of the graph's vertices.
        """
        vertex_list = []
        for i in range(self.v_count):
            vertex_list.append(i)
        return vertex_list

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)

        :return: A list of edges in the graph represented as tuples of the edge's head/tail vertices and weight.
        """
        edge_list = []
        # Iterate through the adjacency matrix to find values > 0 and add those edges to the list.
        for src in range(self.v_count):
            for dst in range(self.v_count):
                if self.adj_matrix[src][dst] > 0:
                    # edge = (source vertex, destination vertex, weight)
                    edge_list.append((src, dst, self.adj_matrix[src][dst]))
        return edge_list

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise

        :param path: A list of vertex indices.
        :return: True if the path is valid; False otherwise.
        """
        # Check whether vertex path[0] exists in the graph.
        if len(path) > 0 and not 0 <= path[0] < self.v_count:
            return False
        # Check connections between each consecutive vertex.
        for i in range(len(path) - 1):
            # If destination vertex does not exist or no directed edge from path[0] to path[1], return False:
            if not 0 <= path[i+1] < self.v_count or self.adj_matrix[path[i]][path[i+1]] < 1:
                return False
        # Valid path.
        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order

        :param v_start: Index of vertex to start search at.
        :param v_end: Index of vertex to end search at. If unspecified, all vertices connected to v_start will be
            visited.
        :return: List of vertices visited, in order visited. If v_end is specified, the search is stopped once it is
            visited.
        """
        # Return empty list if v_start does not exist.
        if not 0 <= v_start < self.v_count:
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
                adj = self.adj_matrix[v]
                # For every vertex connected to v, in descending order:
                for v_adj in range(self.v_count - 1, -1, -1):
                    # If the connected vertex has not been visited, add it to the stack:
                    if adj[v_adj] > 0 and v_adj not in visited:
                        stack.append(v_adj)
        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order

        :param v_start: Index of vertex to start search at.
        :param v_end: Index of vertex to end search at. If unspecified, all vertices connected to v_start will be
            visited.
        :return: List of vertices visited, in order visited. If v_end is specified, the search is stopped once it is
            visited.
        """
        # Return empty list if v_start does not exist.
        if not 0 <= v_start < self.v_count:
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
                adj = self.adj_matrix[v]
                # For every vertex connected to v, in ascending order:
                for v_adj in range(self.v_count):
                    # If the connected vertex has not been visited and is not in the queue, add it to the queue:
                    if adj[v_adj] > 0 and v_adj not in visited and v_adj not in queue:
                        queue.append(v_adj)
        return visited

    def has_cycle(self) -> bool:
        """
        Return True if graph contains a cycle, False otherwise

        :return: True if there is a cycle in the graph. False otherwise.
        """
        visited = []  # Keeps track of visited vertices.
        path = []  # Keeps track of the current path. Use this to determine if a cycle exists.
        # Iterate through every vertex to look for a cycle.
        for v_start in range(self.v_count):
            # If the vertex has not been visited, determine if there is a path starting from it that contains a cycle.
            if v_start not in visited and self._has_cycle(v_start, visited, path) is True:
                return True
        # The graph contains no cycles.
        return False

    def _has_cycle(self, v, visited, path) -> bool:
        """
        Helper method for has_cycle(). Performs DFS using recursion to find a path containing a cycle.

        :param v: Index of the current vertex being visited.
        :param visited: A list of vertex indices already visited.
        :param path: A stack of vertex indices representing the current path in the DFS. Used to determine if a cycle
            exists.
        :return: True if a cycle exists in the graph; False if no cycle exists in the subgraph with 'visited' vertices.
        """
        # Add the current vertex v to visited and to the path.
        visited.append(v)
        path.append(v)
        # Iterate through every vertex v_adj that v connects to:
        for v_adj in [index for index, val in enumerate(self.adj_matrix[v]) if val > 0]:
            # If v_adj has not been visited, determine if there is a path starting from it that contains a cycle.
            if v_adj not in visited and self._has_cycle(v_adj, visited, path) is True:
                return True
            # If v_adj has already been visited and is in the current path, the graph contains a cycle.
            if v_adj in path:
                return True
        # v has no connections that make a cycle in the current path. Remove it from the path:
        path.pop(-1)
        return False

    def dijkstra(self, src: int) -> []:
        """
        Returns a list of the shortest path lengths from a given vertex to all other vertices in the graph, using
            Dijkstra's algorithm.

        :param src: Index of the starting vertex.
        :return: A list of the shortest path lengths from src to all other vertices in the graph.
        """
        # Initialize 'inf' (meaning unreachable) min_dist from src to all vertices.
        min_dist = [float('inf')] * self.v_count
        # Initialize a priority queue with tuple elements (distance, vertex), ordered by ascending distance.
        queue = [(0, src)]
        while len(queue) > 0:
            # Pop/get the (distance, vertex) pair with the shortest distance in the queue.
            d, v = heapq.heappop(queue)
            # If distance from src to v (d) < current record (min_dist[v]):
            if d < min_dist[v]:
                min_dist[v] = d  # Update min_dist to v.
                # For every destination vertex 'v_dst' of an edge connected to v:
                for v_dst in [index for index, val in enumerate(self.adj_matrix[v]) if val > 0]:
                    edge_wgt = self.adj_matrix[v][v_dst]
                    # Push (total_dist to v_dst, v_dst) to priority queue
                    heapq.heappush(queue, (d + edge_wgt, v_dst))
        return min_dist


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')

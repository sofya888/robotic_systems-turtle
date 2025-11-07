import heapq

class PathPlanner:
    def __init__(self, graph:dict):
        self.graph = graph

    def dijkstra(self, start:str, goal:str):
        if start not in self.graph or goal not in self.graph:
            return None
        dist = {n: float('inf') for n in self.graph}
        prev = {n: None for n in self.graph}
        dist[start] = 0
        pq = [(0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == goal: break
            if d > dist[u]: continue
            for v in self.graph[u]:
                nd = d + 1
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path if path and path[0]==start else None

    @staticmethod
    def find_nearest_node(graph:dict, x:int, y:int):
        best, bestd = None, 10**9
        for n in graph:
            nx, ny = map(int, n.split('_'))
            d = abs(nx-x) + abs(ny-y)
            if d < bestd:
                bestd, best = d, n
        return best

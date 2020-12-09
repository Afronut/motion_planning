import heapq


class exercise1():
    """
    docstring
    """

    def __init__(self, s, e, g):
        self.start = s
        self.end = e
        self.v, self.e, self.w = g
        self.e_w = list(zip(self.e, self.w))

    def a_star(self):
        queues = []
        start = list(self.start)
        start = start+[0, self.start[1]]
        end = list(self.end)
        end = end+[0, self.end[1]]
        q = []
        heapq.heappush(q, tuple(start))
        queues.append(q[:])
        top = []
        tops = []
        top = heapq.heappop(q)
        neighbor = self.neighbor_node(top)
        iter_num = 0
        while top[1] is not end[1]:
            while not neighbor:
                if top[1] == end[1]:
                    break
                iter_num += 1
                top = heapq.heappop(q)
                neighbor = self.neighbor_node(top)
            for node in neighbor:
                node.append(top[1])
                heapq.heappush(q, tuple(node))
            queues.append(q)
            tops.append(top)
            neighbor = []
        tops.reverse()
        return queues, tops, iter_num

    def neighbor_node(self, node):
        ret = []
        for edge in self.e_w:
            if node[1] == edge[0][0]:
                ret.append([edge[1], edge[0][1], node[2]+edge[1]])
        node = list(node)
        for i in range(0, len(ret)):
            for nod in self.v:
                if nod[1] == ret[i][1]:
                    ret[i][0] = ret[i][0] + node[2] + nod[0]
        return ret

    def dijkstra(self):
        queues = []
        start = list(self.start)
        start = start+[0, self.start[1]]
        end = list(self.end)
        end = end+[0, self.end[1]]
        q = []
        heapq.heappush(q, tuple(start))
        queues.append(q[:])
        top = []
        unvisited = []
        unvisited.append(start)
        top = heapq.heappop(q)
        neighbor = self.neighbor_node(top)
        iter_num = 0
        while unvisited:
            iter_num += 1
            top = unvisited.pop()
            neighbor = self.neighbor_node(top)
            unvisited += neighbor
            for node in neighbor:
                node.append(top[1])
                heapq.heappush(q, tuple(node))
            queues.append(q[:])
            neighbor = []
        return queues, iter_num

    def a_star_path(self):
        queues, tops, iter_num = self.a_star()
        s = tops[0]
        path = []
        p_len = 0
        while (s[-1] is not "start"):
            s = tops.pop(0)
            p = [s[0], s[1]]
            path.append(p)
            p_len += s[0]
            iter_num += 1
        path.append([0, 'start'])

        return path, iter_num, queues

    def dijkstra_path(self):
        queues, iter_num = self.dijkstra()
        q = float("inf")
        q_g = []
        for que in queues:
            for qu in que:
                iter_num += 1
                if qu[1] == "end" and qu[0] < q:
                    q_g = qu
                    q = qu[0]
        s_q = q_g
        paths = []
        que = queues.pop()
        while s_q[-1] != 'start':
            que = queues.pop()
            q = float("inf")
            for qu in que:
                iter_num += 1
                if qu[1] == s_q[-1] and qu[0] < q:
                    s_q = qu
                    q = qu[0]
            paths.append(s_q)
        paths.reverse()
        paths.append(q_g)
        paths.reverse()

        s = paths[0]
        path = []
        p_len = 0
        while (s[-1] is not "start"):
            s = paths.pop(0)
            p = [s[0], s[1]]
            path.append(p)
            p_len += s[0]
            iter_num += 1
        path.append([0, 'start'])
        return path, iter_num, queues

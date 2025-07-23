import collections

# BFS（幅優先探索）を行う関数
def bfs(graph, start_node):
    """
    グラフを幅優先探索し、訪問した順にノードを出力する関数

    Args:
        graph (dict): 隣接リスト形式のグラフ
        start_node (str): 探索を開始するノード
    """
    visited = set()

    queue = collections.deque([start_node])

    # 開始ノードは最初に訪問済みとして記録する
    visited.add(start_node)

    print("BFSの探索順:")

    # キューが空になるまでループを続ける
    while queue:
        # キューの先頭からノードを1つ取り出す (これが「近い順」の核！)
        current_node = queue.popleft()
        print(current_node, end=" -> ")

        # 取り出したノードに隣接しているノードをすべて調べる
        for neighbor in graph[current_node]:
            # もし隣接ノードがまだ訪問済みでなければ
            if neighbor not in visited:
                # 訪問済みリストに追加
                visited.add(neighbor)
                # キューの末尾に追加
                queue.append(neighbor)

# 1. グラフの定義 (隣接リスト形式)
# キーがノード、バリューがそのノードに隣接するノードのリスト
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D'],
    'C': ['A', 'F', 'G'],
    'D': ['B', 'E'],
    'E': ['D', 'F'],
    'F': ['C', 'E'],
    'G': ['C']
}

# 2. BFSの実行
# 'A'をスタート地点として探索を開始
bfs(graph, 'A')

print("探索終了")
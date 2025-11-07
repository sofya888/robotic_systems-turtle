#!/usr/bin/env python3
import yaml
import numpy as np
from PIL import Image
from rclpy.logging import get_logger

logger = get_logger('map_loader')

class MapLoader:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.graph = None

    def load_map(self, map_yaml_path):
        try:
            with open(map_yaml_path, 'r', encoding='utf-8') as f:
                self.map_info = yaml.safe_load(f)

            img_path = map_yaml_path.replace('.yaml', '.pgm')
            img = Image.open(img_path).convert('L')
            self.map_data = np.array(img)

            self._build_graph_from_map()
            logger.info(f'Карта загружена: {img.size[0]}x{img.size[1]}')
            return True
        except Exception as e:
            logger.error(f'Ошибка загрузки карты: {e}')
            return False

    def _build_graph_from_map(self):
        h, w = self.map_data.shape
        self.graph = {}
        thr = 128
        for y in range(h):
            for x in range(w):
                if self.map_data[y, x] < thr:
                    nid = f'{x}_{y}'
                    neigh = []
                    for dx, dy in ((0,1),(1,0),(0,-1),(-1,0)):
                        nx, ny = x+dx, y+dy
                        if 0 <= nx < w and 0 <= ny < h and self.map_data[ny, nx] < thr:
                            neigh.append(f'{nx}_{ny}')
                    self.graph[nid] = neigh

    def get_graph(self):
        return self.graph

    def get_map_info(self):
        return self.map_info

    def pixel_to_world(self, pixel_x, pixel_y):
        res = float(self.map_info['resolution'])
        h   = int(self.map_data.shape[0])
        ox  = float(self.map_info['origin'][0])
        oy  = float(self.map_info['origin'][1])
        world_x = ox + (pixel_x + 0.5) * res
        world_y = oy + (h - (pixel_y + 0.5)) * res
        return world_x, world_y

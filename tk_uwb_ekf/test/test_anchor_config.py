#!/usr/bin/env python3
"""
Test cases for UWB anchor configuration and backward compatibility
"""

import unittest
import math
import yaml
import tempfile
import os


class TestAnchorConfiguration(unittest.TestCase):
    """Test anchor configuration parsing and usage"""
    
    def test_backward_compatibility_three_anchors(self):
        """Test that existing 3-anchor configuration still works"""
        num_anchors = 3
        anchor_height = 0.0
        tag_height = 0.0
        
        anchor_positions = {}
        test_positions = {
            'anchor_a_pos': [0.0, 0.0],
            'anchor_b_pos': [8.4, 0.0],
            'anchor_c_pos': [7.29, 4.57]
        }
        
        for i in range(num_anchors):
            anchor_label = chr(97 + i)
            param_name = f'anchor_{anchor_label}_pos'
            pos_2d = test_positions.get(param_name, [0.0, 0.0])
            anchor_positions[anchor_label] = list(pos_2d) + [anchor_height] if len(pos_2d) == 2 else list(pos_2d)
        
        self.assertEqual(len(anchor_positions), 3)
        self.assertEqual(anchor_positions['a'], [0.0, 0.0, 0.0])
        self.assertEqual(anchor_positions['b'], [8.4, 0.0, 0.0])
        self.assertEqual(anchor_positions['c'], [7.29, 4.57, 0.0])
    
    def test_five_anchors_with_height(self):
        """Test 5 anchors with height parameters"""
        num_anchors = 5
        anchor_height = 1.5
        tag_height = 0.3
        
        anchor_positions = {}
        test_positions = {
            'anchor_a_pos': [0.0, 0.0],
            'anchor_b_pos': [5.0, 0.0],
            'anchor_c_pos': [2.5, 4.0],
            'anchor_d_pos': [7.0, 3.0],
            'anchor_e_pos': [3.0, 7.0]
        }
        
        for i in range(num_anchors):
            anchor_label = chr(97 + i)
            param_name = f'anchor_{anchor_label}_pos'
            pos_2d = test_positions.get(param_name, [0.0, 0.0])
            anchor_positions[anchor_label] = list(pos_2d) + [anchor_height] if len(pos_2d) == 2 else list(pos_2d)
        
        self.assertEqual(len(anchor_positions), 5)
        self.assertEqual(anchor_positions['e'], [3.0, 7.0, 1.5])
    
    def test_distance_calculation_with_height(self):
        """Test that height affects distance calculation correctly"""
        anchor_pos = [0.0, 0.0, 1.5]  # Anchor at height 1.5m
        tag_height = 0.3
        robot_pos = [2.0, 2.0]
        
        dx = robot_pos[0] - anchor_pos[0]
        dy = robot_pos[1] - anchor_pos[1]
        dz = tag_height - anchor_pos[2]
        
        dist_2d = math.sqrt(dx**2 + dy**2)
        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        
        self.assertAlmostEqual(dist_2d, 2.828, places=2)
        self.assertAlmostEqual(dist_3d, 3.072, places=2)
        self.assertGreater(dist_3d, dist_2d)
    
    def test_anchor_label_generation(self):
        """Test anchor label generation for various counts"""
        test_cases = [
            (3, ['a', 'b', 'c']),
            (5, ['a', 'b', 'c', 'd', 'e']),
            (10, ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j'])
        ]
        
        for num_anchors, expected_labels in test_cases:
            labels = [chr(97 + i) for i in range(num_anchors)]
            self.assertEqual(labels, expected_labels)


class TestYAMLConfiguration(unittest.TestCase):
    """Test YAML configuration file parsing"""
    
    def test_yaml_backward_compatible(self):
        """Test YAML without height parameters (backward compatible)"""
        yaml_content = """anchors:
- name: A0
  x: 0.0
  y: 0.0
- name: A1
  x: 8.4
  y: 0.0
- name: A2
  x: 7.29
  y: 4.57
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_file = f.name
        
        try:
            with open(temp_file, 'r') as f:
                anchor_data = yaml.safe_load(f)
            
            anchors_list = anchor_data.get('anchors', [])
            num_anchors = len(anchors_list)
            anchor_height = anchor_data.get('anchor_height', 0.0)
            tag_height = anchor_data.get('tag_height', 0.0)
            
            self.assertEqual(num_anchors, 3)
            self.assertEqual(anchor_height, 0.0)
            self.assertEqual(tag_height, 0.0)
        finally:
            os.unlink(temp_file)
    
    def test_yaml_with_height_parameters(self):
        """Test YAML with height parameters"""
        yaml_content = """anchors:
- name: A0
  x: 0.0
  y: 0.0
- name: A1
  x: 5.0
  y: 0.0
- name: A2
  x: 2.5
  y: 4.0
- name: A3
  x: 7.0
  y: 3.0
- name: A4
  x: 3.0
  y: 7.0
anchor_height: 1.5
tag_height: 0.3
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_file = f.name
        
        try:
            with open(temp_file, 'r') as f:
                anchor_data = yaml.safe_load(f)
            
            anchors_list = anchor_data.get('anchors', [])
            num_anchors = len(anchors_list)
            anchor_height = anchor_data.get('anchor_height', 0.0)
            tag_height = anchor_data.get('tag_height', 0.0)
            
            self.assertEqual(num_anchors, 5)
            self.assertEqual(anchor_height, 1.5)
            self.assertEqual(tag_height, 0.3)
        finally:
            os.unlink(temp_file)
    
    def test_yaml_with_individual_z_coordinates(self):
        """Test YAML with individual z-coordinates per anchor"""
        yaml_content = """anchors:
- name: A0
  x: 0.0
  y: 0.0
  z: 1.0
- name: A1
  x: 5.0
  y: 0.0
  z: 1.5
- name: A2
  x: 2.5
  y: 4.0
  z: 2.0
anchor_height: 1.5
tag_height: 0.3
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_file = f.name
        
        try:
            with open(temp_file, 'r') as f:
                anchor_data = yaml.safe_load(f)
            
            anchors_list = anchor_data.get('anchors', [])
            
            anchor_params = {}
            for i, anchor in enumerate(anchors_list):
                param_name = f'anchor_{chr(97 + i)}_pos'
                if 'z' in anchor:
                    anchor_params[param_name] = [anchor['x'], anchor['y'], anchor['z']]
                else:
                    anchor_params[param_name] = [anchor['x'], anchor['y']]
            
            self.assertEqual(anchor_params['anchor_a_pos'], [0.0, 0.0, 1.0])
            self.assertEqual(anchor_params['anchor_b_pos'], [5.0, 0.0, 1.5])
            self.assertEqual(anchor_params['anchor_c_pos'], [2.5, 4.0, 2.0])
        finally:
            os.unlink(temp_file)


if __name__ == '__main__':
    unittest.main()

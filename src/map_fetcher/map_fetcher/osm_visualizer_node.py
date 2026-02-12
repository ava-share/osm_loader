#!/usr/bin/env python3
"""ROS 2 node for visualizing OpenStreetMap data with detailed road network information.

This node parses OSM XML files and creates interactive matplotlib visualizations showing:
- Color-coded road types (motorway, primary, residential, etc.)
- Speed limits and lane counts annotated on major roads
- Traffic signals and pedestrian crossings marked with symbols
- Surface quality color-coding (paved vs unpaved)
- One-way streets shown with dashed lines
- Turn lane information
- Buildings and structures

The visualization includes comprehensive statistics about the map data.

Parameters:
    osm_file (str): Full path to the OSM XML file to visualize

Example:
    ros2 run map_fetcher osm_visualizer_node --ros-args -p osm_file:=/path/to/map.osm
"""
from __future__ import annotations

import os
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node


class OSMVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('osm_visualizer_node')
        self.declare_parameter('osm_file', '')
        
        osm_file = self.get_parameter('osm_file').get_parameter_value().string_value
        if not osm_file or not Path(osm_file).exists():
            self.get_logger().error(f'OSM file not found: {osm_file}')
            return
            
        self.visualize_osm(osm_file)
    
    def visualize_osm(self, osm_file: str) -> None:
        """Parse and visualize an OSM file using matplotlib."""
        self.get_logger().info(f'Visualizing OSM file: {osm_file}')
        
        tree = ET.parse(osm_file)
        root = tree.getroot()
        
        # Parse all nodes (points with coordinates)
        nodes = {}
        traffic_signals = []
        crossings = []
        
        for node in root.findall('node'):
            node_id = node.get('id')
            lat = node.get('lat')
            lon = node.get('lon')
            if node_id and lat and lon:
                coord = (float(lon), float(lat))
                nodes[node_id] = coord
                
                # Check for traffic signals and crossings
                tags = {tag.get('k'): tag.get('v') for tag in node.findall('tag')}
                if tags.get('highway') == 'traffic_signals':
                    traffic_signals.append(coord)
                elif tags.get('highway') == 'crossing' or tags.get('crossing'):
                    crossings.append(coord)
        
        # Create figure with more space for legend
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # Statistics counters
        way_count = 0
        stats = {
            'oneway': 0,
            'with_maxspeed': 0,
            'with_lanes': 0,
            'with_turn_lanes': 0,
            'buildings': 0,
            'traffic_signals': len(traffic_signals),
            'crossings': len(crossings),
            'surface_paved': 0,
            'surface_unpaved': 0,
            'surface_unknown': 0
        }
        
        # Plot ways with different colors based on attributes
        for way in root.findall('way'):
            coords = []
            for nd in way.findall('nd'):
                node_ref = nd.get('ref')
                if node_ref in nodes:
                    coords.append(nodes[node_ref])
            
            if not coords:
                continue
            
            # Extract tags
            tags = {tag.get('k'): tag.get('v') for tag in way.findall('tag')}
            
            lons, lats = zip(*coords)
            
            # Determine way type and styling
            highway = tags.get('highway', '')
            building = tags.get('building', '')
            maxspeed = tags.get('maxspeed', '')
            lanes = tags.get('lanes', '')
            oneway = tags.get('oneway', '')
            surface = tags.get('surface', '')
            turn_lanes = tags.get('turn:lanes', '')
            
            # Track surface statistics
            if surface:
                if surface in ['asphalt', 'concrete', 'paved', 'cement']:
                    stats['surface_paved'] += 1
                elif surface in ['gravel', 'dirt', 'ground', 'sand', 'compacted', 'unpaved']:
                    stats['surface_unpaved'] += 1
            else:
                stats['surface_unknown'] += 1
            
            # Track turn lanes
            if turn_lanes:
                stats['with_turn_lanes'] += 1
            
            # Choose color and width based on type and attributes
            if building:
                color = 'gray'
                linewidth = 0.5
                alpha = 0.4
                label = 'Building'
                stats['buildings'] += 1
            elif highway in ['motorway', 'trunk']:
                # Color by surface quality if available
                if surface in ['gravel', 'dirt', 'ground', 'sand', 'unpaved']:
                    color = 'brown'
                elif surface in ['asphalt', 'concrete', 'paved']:
                    color = 'darkblue'
                else:
                    color = 'red'
                linewidth = 2.5 if turn_lanes else 2.0
                alpha = 0.8
                label = f'Major road'
            elif highway in ['primary', 'secondary']:
                if surface in ['gravel', 'dirt', 'ground', 'sand', 'unpaved']:
                    color = 'brown'
                elif surface in ['asphalt', 'concrete', 'paved']:
                    color = 'darkblue'
                else:
                    color = 'orange'
                linewidth = 2.0 if turn_lanes else 1.5
                alpha = 0.8
                label = f'Primary/Secondary'
            elif highway in ['residential', 'tertiary']:
                if surface in ['gravel', 'dirt', 'ground', 'sand', 'unpaved']:
                    color = 'brown'
                else:
                    color = 'yellow'
                linewidth = 1.5 if turn_lanes else 1.0
                alpha = 0.7
                label = 'Residential'
            elif highway:
                if surface in ['gravel', 'dirt', 'ground', 'sand', 'unpaved']:
                    color = 'brown'
                else:
                    color = 'lightblue'
                linewidth = 1.2 if turn_lanes else 0.8
                alpha = 0.6
                label = 'Other road'
            else:
                color = 'lightgray'
                linewidth = 0.5
                alpha = 0.4
                label = 'Other'
            
            # Adjust style for oneway streets
            if oneway in ['yes', '1', 'true']:
                linestyle = '--'
                stats['oneway'] += 1
            else:
                linestyle = '-'
            
            # Track statistics
            if maxspeed:
                stats['with_maxspeed'] += 1
            if lanes:
                stats['with_lanes'] += 1
            
            ax.plot(lons, lats, color=color, linewidth=linewidth, 
                   alpha=alpha, linestyle=linestyle)
            
            # Annotate with details for major roads
            if highway in ['motorway', 'trunk', 'primary', 'secondary'] and (maxspeed or lanes):
                mid_idx = len(lons) // 2
                annotation = []
                if maxspeed:
                    annotation.append(f'{maxspeed}')
                if lanes:
                    annotation.append(f'{lanes}L')
                if annotation:
                    ax.annotate(' '.join(annotation), 
                              xy=(lons[mid_idx], lats[mid_idx]),
                              fontsize=6, color='darkred',
                              bbox=dict(boxstyle='round,pad=0.3', 
                                      facecolor='white', alpha=0.7))
            
            way_count += 1
        
        # Plot traffic signals as red triangles
        if traffic_signals:
            signal_lons, signal_lats = zip(*traffic_signals)
            ax.scatter(signal_lons, signal_lats, c='red', marker='^', s=100, 
                      label='Traffic Signals', zorder=5, edgecolors='darkred', linewidths=1.5)
        
        # Plot crossings as green circles
        if crossings:
            cross_lons, cross_lats = zip(*crossings)
            ax.scatter(cross_lons, cross_lats, c='green', marker='o', s=50, 
                      label='Crossings', zorder=5, edgecolors='darkgreen', linewidths=1)
        
        # Create custom legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', linewidth=2, label='Major roads (motorway/trunk)'),
            Line2D([0], [0], color='orange', linewidth=1.5, label='Primary/Secondary roads'),
            Line2D([0], [0], color='yellow', linewidth=1, label='Residential roads'),
            Line2D([0], [0], color='lightblue', linewidth=0.8, label='Other roads'),
            Line2D([0], [0], color='darkblue', linewidth=2, label='Paved roads'),
            Line2D([0], [0], color='brown', linewidth=1.5, label='Unpaved roads'),
            Line2D([0], [0], color='gray', linewidth=0.5, label='Buildings'),
            Line2D([0], [0], color='black', linewidth=1, linestyle='--', label='One-way streets'),
            Line2D([0], [0], marker='^', color='w', markerfacecolor='red', 
                  markersize=8, label='Traffic Signals'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                  markersize=6, label='Crossings'),
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
        
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        
        # Enhanced title with statistics
        title_lines = [
            f'OpenStreetMap: {Path(osm_file).name}',
            f'Nodes: {len(nodes)} | Ways: {way_count} | Buildings: {stats["buildings"]}',
            f'One-way: {stats["oneway"]} | Speed limits: {stats["with_maxspeed"]} | Lane info: {stats["with_lanes"]} | Turn lanes: {stats["with_turn_lanes"]}',
            f'Traffic signals: {stats["traffic_signals"]} | Crossings: {stats["crossings"]} | Surface (Paved/Unpaved/Unknown): {stats["surface_paved"]}/{stats["surface_unpaved"]}/{stats["surface_unknown"]}'
        ]
        ax.set_title('\n'.join(title_lines), fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        self.get_logger().info(f'Displaying map with {len(nodes)} nodes, {way_count} ways, '
                              f'{stats["oneway"]} one-way streets, {stats["with_maxspeed"]} '
                              f'with speed limits, {stats["with_lanes"]} with lane info, '
                              f'{stats["with_turn_lanes"]} with turn lanes, '
                              f'{stats["traffic_signals"]} traffic signals, {stats["crossings"]} crossings')
        plt.tight_layout()
        plt.show()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OSMVisualizerNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

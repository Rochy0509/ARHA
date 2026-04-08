#!/usr/bin/env python3
import os
import struct

def convert_to_binary(filepath):
    print(f"Reading ASCII STL: {filepath}")
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    facets = []
    current_facet = None
    
    for line in lines:
        parts = line.split()
        if not parts: continue
        
        if parts[0] == 'facet':
            current_facet = {'normal': [float(parts[2]), float(parts[3]), float(parts[4])], 'vertices': []}
        elif parts[0] == 'vertex':
            current_facet['vertices'].append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == 'endfacet':
            facets.append(current_facet)
            
    print(f"  -> Found {len(facets)} triangles. Writing binary...")
    
    with open(filepath + '.tmp', 'wb') as f:
        f.write(b'Binary STL converted by Python script' + b'\0' * 43) # 80 byte header
        f.write(struct.pack('<I', len(facets)))
        
        for facet in facets:
            f.write(struct.pack('<3f', *facet['normal']))
            for v in facet['vertices']:
                f.write(struct.pack('<3f', *v))
            f.write(struct.pack('<H', 0)) # Attribute byte count
            
    os.rename(filepath + '.tmp', filepath)
    print(f"  -> Successfully converted {filepath} to binary!\n")

def main():
    directory = os.path.dirname(os.path.abspath(__file__))
    for filename in os.listdir(directory):
        if filename.endswith('.stl') or filename.endswith('.STL'):
            filepath = os.path.join(directory, filename)
            
            with open(filepath, 'rb') as f:
                header = f.read(5)
                
            if header == b'solid':
                convert_to_binary(filepath)
            else:
                print(f"Skipping {filename}: already binary.")

if __name__ == '__main__':
    main()

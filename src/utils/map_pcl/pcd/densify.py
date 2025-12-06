import numpy as np
import re

def read_pcd(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    header = []
    points = []
    is_data = False

    for line in lines:
        if is_data:
            parts = line.strip().split()
            if len(parts) >= 3:
                x, y, z = map(float, parts[:3])
                points.append([x, y, z])
        else:
            header.append(line)
            if re.match(r"^DATA\s+ascii$", line.strip(), re.IGNORECASE):
                is_data = True

    return header, np.array(points)

def write_pcd(file_path, header, points):
    with open(file_path, 'w') as file:
        for line in header:
            if line.startswith('POINTS'):
                file.write(f'POINTS {len(points)}\n')
            else:
                file.write(line)
        
        for point in points:
            file.write(f'{point[0]} {point[1]} {point[2]}\n')

def densify_pcd(input_filename, output_filename, z_factor=2):
    header, points = read_pcd(input_filename)
    
    new_points = []
    for point in points:
        new_points.append(point)
        for _ in range(z_factor - 1):
            offset = np.random.uniform(-0.01, 0.01)
            new_point = point + [0, 0, offset]
            new_points.append(new_point)
    
    write_pcd(output_filename, header, new_points)

if __name__ == "__main__":
    input_filename = 'res.pcd'
    output_filename = 'res.pcd'
    z_factor = 2

    densify_pcd(input_filename, output_filename, z_factor)


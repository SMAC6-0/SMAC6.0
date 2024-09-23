from stl import mesh

# Load your STL file
your_mesh = mesh.Mesh.from_file('/Users/canguven/Downloads/empire.stl')

# Scale the mesh (replace 0.5, 0.5, 0.5 with your desired scale factors)
your_mesh.x *= 0.8
your_mesh.y *= 0.8
your_mesh.z *= 0.8

# Save the scaled STL
your_mesh.save('/Users/canguven/Downloads/empire2.stl')

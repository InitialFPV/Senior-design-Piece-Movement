# board_lookup_gen.py
xo = 10  # mm offset for X
yo = 10  # mm offset for Y

# Define X coordinates for 12 columns (center positions)
x_positions = [xo]
x = xo
for col in range(11):  # 0→10
    if col in [0, 10]:
        dx = 30.0
    elif col in [1, 9]:
        dx = 45.09
    else:
        dx = 37.0
    x += dx
    x_positions.append(x)

# Define Y coordinates for 8 rows
y_positions = [yo + 37.0 * row for row in range(8)]

# Generate C++ array
print("constexpr float board_pos[12][8][2] = {")
for a in range(12):
    print("    {", end="")
    for b in range(8):
        print(f"{{{x_positions[a]:.3f}f, {y_positions[b]:.3f}f}}", end="")
        if b != 7:
            print(", ", end="")
    print("}", end="")
    if a != 11:
        print(",")
    else:
        print()
print("};")

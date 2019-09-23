with open("./mobility_25_users.tcl") as f:
    with open("./mobility_5am_7am.tcl", "w") as out:
        lines = [i.split() for i in f.readlines() if float(i.split()[2]) > 18000]
        for i, v in enumerate(lines):
            lines[i][2] = str( float(lines[i][2]) - 18000 )
            lines[i].append("\n")
            out.write(" ".join(lines[i]))

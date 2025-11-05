'  # 'sym' or 'num'
    joint_types = [ 'r', 'p', 'p', 'r', 'r', 'r' ]  #joint types - 'r' for revolute, 'p' for prismatic
    
    thetas = [ sp.symbols('theta_1'), 0, 0, sp.symbols('theta_4'), sp.symbols('theta_5'), sp.symbols('theta_6') ]
    ds = [ sp.symbols('d_1'), sp.symbols('d_2'), sp.symbols('d_3'), 0, 0, sp.symbols('d_6') ]
    ase = [ 0, 0, 0, 0, 0, 0 ]
    alphas = [ 0, -sp.pi
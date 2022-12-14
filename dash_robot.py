from dash import Dash, dcc, html, Input, Output, ALL, State, MATCH, ALLSMALLER, ctx
import plotly.express as px
import plotly.graph_objects as go
from visual_kinematics.RobotSerial import *
from math import pi
import numpy as np

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
np.set_printoptions(precision=5, suppress=True)

app = Dash(__name__, external_stylesheets=external_stylesheets)
server = app.server


# draw arrows
def drawArrow(start, end, color, w, cone_width, arrow_dir):
    print("draw arrow")
    x = [start[0], end[0]]
    y = [start[1], end[1]]
    z = [start[2], end[2]]
    return [
        go.Scatter3d(
            x = [x[0], x[0]+0.95*(x[1]-x[0])],
            y = [y[0], y[0]+0.95*(y[1]-y[0])],
            z = [z[0], z[0]+0.95*(z[1]-z[0])],
            mode="lines",
            line = dict(color=color, width=w),
            showlegend = False,
            name = arrow_dir
        ),
        go.Cone(
            x=[x[1]], y=[y[1]], z=[z[1]], 
            u=[0.3*(x[1]-x[0])], v=[0.3*(y[1]-y[0])], w=[0.3*(z[1]-z[0])],
            anchor = "tip",
            colorscale = [[0, color], [1, color]],
            showscale = False,
            showlegend = False,
            sizemode = "absolute",
            sizeref = cone_width,
            name = arrow_dir
        )
    ]

# draw orientations
def drawMat(mat, lw, scale, cone):
    print("draw mat")
    arrow_x = drawArrow([mat[0, 3], mat[1, 3], mat[2, 3]], [mat[0, 3]+mat[0, 0]*scale, mat[1, 3]+mat[1, 0]*scale, mat[2, 3]+mat[2, 0]*scale], "red", lw, cone, "x")
    arrow_y = drawArrow([mat[0, 3], mat[1, 3], mat[2, 3]], [mat[0, 3]+mat[0, 1]*scale, mat[1, 3]+mat[1, 1]*scale, mat[2, 3]+mat[2, 1]*scale], "green", lw, cone, "y")
    arrow_z = drawArrow([mat[0, 3], mat[1, 3], mat[2, 3]], [mat[0, 3]+mat[0, 2]*scale, mat[1, 3]+mat[1, 2]*scale, mat[2, 3]+mat[2, 2]*scale], "blue", lw, cone, "z")
    xy = arrow_x + arrow_y
    return xy + arrow_z

# draw the robot
def draw_robot(robot, height, w_link, w_joint, w_arrow, l_arrow, c_arrow, l_color):
    print("draw robot")
    poses = [[0, 0, 0]]
    matrices = [np.eye(4)]
    transitional_matrices = []
    for i in range(robot.num_axis):
        poses.append(list(robot.axis_frames[i][:3, 3]))
        matrices.append(np.array(robot.axis_frames[i][:4, :4]))
    poses = np.array(poses)
    # print("\npositions: \n")
    # print(poses)
    # print("\nfinal matrices: \n")
    # print(matrices)
    # print("\nTransitional matrices:\n")
    for i in range(robot.num_axis):
        print(np.array(robot.ts[i][:4, :4]))
        transitional_matrices.append(np.array(robot.ts[i][:4, :4]))

    layout = go.Layout(scene=dict(aspectmode="data"), height = max(10, int(height)), margin=dict(l=0, r=0, b=0, t=0))
    data=[go.Scatter3d(x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],mode='lines+markers', line=dict(color=l_color, width=int(w_link)), marker=dict(color='black', size=int(w_joint)), showlegend = False, name="robot joint")]
    for mat in matrices:
        data = data + drawMat(mat, int(w_arrow), float(l_arrow), float(c_arrow))
    fig = go.Figure(data=data, layout=layout)

    return fig, matrices, transitional_matrices

# layout
app.layout = html.Div([
    html.H1("Model your own robot using DH parameters"),
    html.Div([
        "This website allows you to model a robot based on DH parameters. Alternatively, you can also download the code and locally run the python script on your computer.", html.Br(),
        dcc.Link(html.Button('go to git repository', style={'fontSize': '1.1em'}), href="https://github.com/schulzch19/RMMI1_dash_robot", refresh = True)
    ]),
    html.H2("Instructions"),
    html.Ul([
        html.Li("Settings: Adapt the plot to your personal preferences. Play around and see what happens."),
        html.Li([
            "Robot modelling",
            html.Ol([
                html.Li("Set the number of joints, the table below will automatically change to the proper number of rows."),
                html.Li("Set the parameters: Each row represents one joint (d, a, alpha, theta). If you are unsure about how the parameters influence the model, look at the graph.")
                ], style={"list-style-position": "outside"})
            ]),
        html.Li(["Move the robot's TCP", html.Br(), "All joints are automatically set as revolute joints. The last column gives you the possibility to set the joint's angle and \
                thereby move the robot around. Since the server is rather slow, I recommend you don't use the arrows some browsers provide, but instead immediately set the \"angle\" column to the desired value. \
                TCP position and orientation are automatically updated und shown underneath the table."]),
        html.Li([
            "Inverse Kinematics: This script comes with an implementation of inverse kinematics.",
            html.Ol([
                html.Li("The two rows TCP & Rotation now function as an input. Set the position you wish the robot to move to in cartesian coordinates, \
                    set the orientation using quaternions."),
                html.Li("Click the \"CALCULATE INVERSE\" button. The angles in the table are now automatically updated. \
                    Usually, the found solution will slightly deviate from the requested position. In this case, position & orientation will automatically be updated.")
                ], style={"list-style-position": "outside"})
            ]),
        html.Li("If you are using this visualization tool to further understand kinematic models, all matrices for the current position are printed underneath the plot.")
    ], style={"list-style-position": "outside", "margin-left": "2.5rem"}),
    html.H2("Settings"),
    html.Div([
        " fig height: ",
        dcc.Input(id='fig_height', value='500', type='number', style={'width': '5em'}), html.Br(),
        " link width: ", 
        dcc.Input(id='width_link', value='40', type='number', style={'width': '5em'}),  html.Br(),
        " joint width: ", 
        dcc.Input(id='width_joint', value='15', type='number', style={'width': '5em'}),  html.Br(),
        " arrow width: ", 
        dcc.Input(id='width_arrow', value='5', type='number', style={'width': '5em'}),  html.Br(),
        " arrow length: ",
        dcc.Input(id='length_arrow', value='0.5', type='number', style={'width': '5em'}), html.Br(),
        " arrow cone size: ",
        dcc.Input(id='size_cone_arrow', value='0.1', type='number', style={'width': '5em'}), html.Br(),
        " link color (e.g. teal, #f25c19): ",
        dcc.Input(id='color_link', value='teal', style={'width': '7em'})

    ]),
    html.H2("DH parameters"),
    html.Div(["number of joints: ", dcc.Input(id='nbr_joints', value='6', type='number', style={'width': '5em'})], style={"fontWeight": "bold"}), 
    html.Table([

        html.Thead(
            html.Tr([html.Th("joint", style={"text-align": "center"}), html.Th("d", style={"text-align": "center"}), html.Th("a", style={"text-align": "center"}), html.Th("alpha [??]", style={"text-align": "center"}), html.Th("theta [??]", style={"text-align": "center"}), html.Th("angle [??]", style={"text-align": "center"})])
        ),
        html.Tbody([
            html.Tr([
                    html.Th(1, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd1', "type": "dyn-in"}, value=0.78, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a1', "type": "dyn-in"}, value=0.41, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha1', "type": "dyn-in"}, value=-90, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta1', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle1', "type": "dyn-in-angle"}, value=0, type='number', style={'width': '5em'}))
                    ]),
            html.Tr([
                    html.Th(2, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd2', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a2', "type": "dyn-in"}, value=1.075, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha2', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta2', "type": "dyn-in"}, value=-90, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle2', "type": "dyn-in-angle"}, value=0, type='number', style={'width': '5em'}))
                    ]),
            html.Tr([
                    html.Th(3, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd3', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a3', "type": "dyn-in"}, value=0.165, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha3', "type": "dyn-in"}, value=-90, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta3', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle3', "type": "dyn-in-angle"}, value=0, type='number', style={'width': '5em'}))
                    ]),
            html.Tr([
                    html.Th(4, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd4', "type": "dyn-in"}, value=1.056, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a4', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha4', "type": "dyn-in"}, value=90, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta4', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle4', "type": "dyn-in-angle"}, value=0, type='number', style={'width': '5em'}))
                    ]),
            html.Tr([
                    html.Th(5, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd5', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a5', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha5', "type": "dyn-in"}, value=-90, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta5', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle5', "type": "dyn-in-angle"}, value=0, type='number', style={'width': '5em'}))
                    ]),
            html.Tr([
                    html.Th(6, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd6', "type": "dyn-in"}, value=0.25, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a6', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha6', "type": "dyn-in"}, value=0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta6', "type": "dyn-in"}, value=180, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle6', "type": "dyn-in-angle"}, value=0, type='number', style={'width': '5em'}))
                    ])        
        ]),

    ], id='joints_table'),
    html.Br(),
    html.Div([
        "TCP: ",
        dcc.Input(id='pos-x', value=0, type='number', style={'width': '7em'}),
        " x, ",
        dcc.Input(id='pos-y', value=0, type='number', style={'width': '7em'}),
        " y, ",
        dcc.Input(id='pos-z', value=0, type='number', style={'width': '7em'}),
        " z",
    ], style={'marginBottom': "1rem"}),
    html.Div([
        "Rotation: ",
        dcc.Input(id='rot-i', value=0, type='number', style={'width': '7em'}),
        " i + ",
        dcc.Input(id='rot-j', value=0, type='number', style={'width': '7em'}),
        " j + ",
        dcc.Input(id='rot-k', value=0, type='number', style={'width': '7em'}),
        " k + ",
        dcc.Input(id='rot-w', value=0, type='number', style={'width': '7em'}),
    ]),
    html.Br(),
    html.Div([
        html.Button('Calculate Inverse', id='inverse-btn', n_clicks=0, style={'fontSize': '1.1em'}),
        html.Button('Reset Angles', id='angle-reset-btn', n_clicks=0, style={'fontSize': '1.1em'}),
        html.Button('Reset DH parameters', id='dh-reset-btn', n_clicks=0, style={'fontSize': '1.1em'}),
    ]),
    html.H2("Visualization"),
    dcc.Graph(id='graph-robot'),
    html.Div([
        html.H2("Transformation matrices"),
        html.Table([], id='mats_table', style={'fontSize':25})
    ])

], style={'marginLeft': "1rem", 'fontSize': 20})

# update graph and printed matrices
@app.callback(
    Output('graph-robot', 'figure'),
    Output('pos-x', 'value'),
    Output('pos-y', 'value'),
    Output('pos-z', 'value'),
    Output('rot-i', 'value'),
    Output('rot-j', 'value'),
    Output('rot-k', 'value'),
    Output('rot-w', 'value'),
    Output('mats_table', 'children'),
    Input(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
    Input(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
    Input(component_id='fig_height', component_property='value'),
    Input(component_id='width_link', component_property='value'),
    Input(component_id='width_joint', component_property='value'),
    Input(component_id='width_arrow', component_property='value'),
    Input(component_id='length_arrow', component_property='value'),
    Input(component_id='size_cone_arrow', component_property='value'),
    Input(component_id='color_link', component_property='value')
)
def update_robot(params, angles, h, w_link, w_joint, w_arrow, l_arrow, c_arrow, l_color):
    print("update robot")
    triggered_id = ctx.triggered_id
    dh_params = []
    for l in range(0, int(len(params)/4)):
        axis = [float(n) for n in params[4*l:4*l+4]]
        dh_params.append([axis[0], axis[1], axis[2]/180*pi, axis[3]/180*pi])
    dh_params = np.array(dh_params)
    print(dh_params)
    robot = RobotSerial(dh_params)
    robot_angles = np.array([float(a)/180*pi for a in angles])
    f = robot.forward(robot_angles)
    
    fig, matrices, matr_trans = draw_robot(robot, h, w_link, w_joint, w_arrow, l_arrow, c_arrow, l_color)


    # fig.update_layout(transition_duration=500)
    pos = f.t_3_1.reshape([3, ])
    quat = f.q_4

    return fig, \
        "{:.4f}".format(pos[0]), "{:.4f}".format(pos[1]), "{:.4f}".format(pos[2]), \
        "{:.4f}".format(quat[0]), "{:.4f}".format(quat[1]), "{:.4f}".format(quat[2]), "{:.4f}".format(quat[3]), \
        [html.Thead(
            html.Tr([html.Th("joint", style={"text-align": "center"}), html.Th("Transition joint (i-1) to joint i", style={"text-align": "center"}), html.Th("Transition up to joint i", style={"text-align": "center"})])
        ),
        html.Tbody([
            html.Tr([
                    html.Th(i, style={"text-align": "center"}), 
                    html.Td(html.Div([
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matr_trans[i-1][0][0], 3), round(matr_trans[i-1][0][1], 3), round(matr_trans[i-1][0][2], 3), round(matr_trans[i-1][0][3], 3)), html.Br(), 
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matr_trans[i-1][1][0], 3), round(matr_trans[i-1][1][1], 3), round(matr_trans[i-1][1][2], 3), round(matr_trans[i-1][1][3], 3)), html.Br(),
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matr_trans[i-1][2][0], 3), round(matr_trans[i-1][2][1], 3), round(matr_trans[i-1][2][2], 3), round(matr_trans[i-1][2][3], 3)), html.Br(),
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matr_trans[i-1][3][0], 3), round(matr_trans[i-1][3][1], 3), round(matr_trans[i-1][3][2], 3), round(matr_trans[i-1][3][3], 3))
                        ], style={"white-space": "pre", "font-family": "monospace"})),
                    html.Td(html.Div([
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matrices[i][0][0], 3), round(matrices[i][0][1], 3), round(matrices[i][0][2], 3), round(matrices[i][0][3], 3)), html.Br(), 
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matrices[i][1][0], 3), round(matrices[i][1][1], 3), round(matrices[i][1][2], 3), round(matrices[i][1][3], 3)), html.Br(),
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matrices[i][2][0], 3), round(matrices[i][2][1], 3), round(matrices[i][2][2], 3), round(matrices[i][2][3], 3)), html.Br(),
                        "{1:>{0}.3f}|{2:>{0}.3f}|{3:>{0}.3f}|{4:>{0}.3f}".format(8, round(matrices[i][3][0], 3), round(matrices[i][3][1], 3), round(matrices[i][3][2], 3), round(matrices[i][3][3], 3))
                        ], style={"white-space": "pre", "font-family": "monospace"})),
                    ]) for i in range(1,len(matrices))]
        )]

# set angles to 0 or calculate the inverse kinematics
@app.callback(
    Output(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
    Input('inverse-btn', 'n_clicks'),
    Input('angle-reset-btn', 'n_clicks'),
    State(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
    State('pos-x', 'value'),
    State('pos-y', 'value'),
    State('pos-z', 'value'),
    State('rot-i', 'value'),
    State('rot-j', 'value'),
    State('rot-k', 'value'),
    State('rot-w', 'value'),
    State(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
)
def calc_inverse(n_c_inv, n_c_res, params, x, y, z, i, j, k, w, angles):
    print("calc inverse")
    triggered_id = ctx.triggered_id
    print(triggered_id)
    if triggered_id == None:
        print("none detected")
        return angles
    else:
        dh_params = []
        for l in range(0, int(len(params)/4)):
            axis = [float(n) for n in params[4*l:4*l+4]]
            dh_params.append([axis[0], axis[1], axis[2]/180*pi, axis[3]/180*pi])
        dh_params = np.array(dh_params)
        print(dh_params)
        robot = RobotSerial(dh_params)

        if triggered_id == 'inverse-btn':
            print("inverse btn")
            xyz = np.array([[x], [y], [z]])
            quat = np.array([i, j, k, w])
            end = Frame.from_q_4(quat, xyz)
            robot.inverse(end)
            angles = robot.axis_values

            return tuple(["{:.2f}".format(a/pi*180) for a in angles])
        elif triggered_id == 'angle-reset-btn':
            print("reset angles")

            angles = [0 for i in range(robot.num_axis)]

            return tuple(["{:.2f}".format(a/pi*180) for a in angles])
        
        else:
            print('\n\n\n\n\n\n inv btn error \n\n\n\n\n\n')
            angles = [5 for i in range(robot.num_axis)]

            return tuple(["{:.2f}".format(a/pi*180) for a in angles])

# reset dh parameters
@app.callback(
    Output(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
    Input('dh-reset-btn', 'n_clicks'),
    State('nbr_joints', 'value'),
    State(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
)
def reset_dh(n_c, nbr_joints, params):
    print('reset_dh')
    triggered_id = ctx.triggered_id
    print(triggered_id)
    if triggered_id == None:
        print("none detected")
        return params
    else:
        if triggered_id == 'dh-reset-btn':
            return [0 for _ in range(int(nbr_joints)*4)]
        else:
            print('\n\n\n\n\n\n reset dh error \n\n\n\n\n\n')
            return [12 for _ in range(int(nbr_joints)*4)]

# change number of joints
@app.callback(
    Output(component_id='joints_table', component_property='children'),
    Output(component_id='nbr_joints', component_property='value'),
    Input(component_id='nbr_joints', component_property='value'),
    State(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
    State(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
)
def update_output_div(joints, params, angles):
    print("update output div")
    # print("params+angles")
    # print(params)
    # print(angles)
    return [
        html.Thead(
            html.Tr([html.Th("joint", style={"text-align": "center"}), html.Th("d", style={"text-align": "center"}), html.Th("a", style={"text-align": "center"}), html.Th("alpha [??]", style={"text-align": "center"}), html.Th("theta [??]", style={"text-align": "center"}), html.Th("angle [??]", style={"text-align": "center"})])
        ),
        html.Tbody([
            html.Tr([
                    html.Th(i, style={"text-align": "center"}), 
                    html.Td(dcc.Input(id={"name": 'd' + str(i), "type": "dyn-in"}, value=params[4*(i-1)] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a' + str(i), "type": "dyn-in"}, value=params[4*(i-1)+1] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha' + str(i), "type": "dyn-in"}, value=params[4*(i-1)+2] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta' + str(i), "type": "dyn-in"}, value=params[4*(i-1)+3] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle' + str(i), "type": "dyn-in-angle"}, value=angles[(i-1)] if (i-1) < len(angles) else 0, type='number', style={'width': '5em'}))
                    ]) for i in range(1,max(2, int(joints)+1))
        ]),

    ], max(1, int(joints))

if __name__ == '__main__':
    app.run_server(debug=True)

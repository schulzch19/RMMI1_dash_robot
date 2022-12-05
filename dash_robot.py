from dash import Dash, dcc, html, Input, Output, ALL, State, MATCH, ALLSMALLER, ctx
from dash_extensions.enrich import Output, DashProxy, Input, MultiplexerTransform, State
import plotly.express as px
import plotly.graph_objects as go
from visual_kinematics.RobotSerial import *
from math import pi
import numpy as np

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
np.set_printoptions(precision=3, suppress=True)

# app = Dash(__name__, external_stylesheets=external_stylesheets)
# app = Dash(__name__)
app = DashProxy(prevent_initial_callbacks=False, transforms=[MultiplexerTransform()], external_stylesheets=external_stylesheets)

robot = None

def drawArrow(start, end, color, w, cone_width):
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
            showlegend = False
        ),
        go.Cone(
            x=[x[1]], y=[y[1]], z=[z[1]], 
            u=[0.3*(x[1]-x[0])], v=[0.3*(y[1]-y[0])], w=[0.3*(z[1]-z[0])],
            anchor = "tip",
            colorscale = [[0, color], [1, color]],
            showscale = False,
            showlegend = False,
            sizemode = "absolute",
            sizeref = cone_width
        )
    ]

def drawMat(mat, lw, scale, cone):
    print("draw mat")
    arrow_x = drawArrow([mat[0, 3], mat[1, 3], mat[2, 3]], [mat[0, 3]+mat[0, 0]*scale, mat[1, 3]+mat[1, 0]*scale, mat[2, 3]+mat[2, 0]*scale], "red", lw, cone)
    arrow_y = drawArrow([mat[0, 3], mat[1, 3], mat[2, 3]], [mat[0, 3]+mat[0, 1]*scale, mat[1, 3]+mat[1, 1]*scale, mat[2, 3]+mat[2, 1]*scale], "green", lw, cone)
    arrow_z = drawArrow([mat[0, 3], mat[1, 3], mat[2, 3]], [mat[0, 3]+mat[0, 2]*scale, mat[1, 3]+mat[1, 2]*scale, mat[2, 3]+mat[2, 2]*scale], "blue", lw, cone)
    xy = arrow_x + arrow_y
    return xy + arrow_z

def draw_robot(robot, height, w_link, w_joint, w_arrow, l_arrow, c_arrow):
    print("draw robot")
    poses = [[0, 0, 0]]
    matrices = [np.eye(4)]
    transitional_matrices = []
    for i in range(robot.num_axis):
        poses.append(list(robot.axis_frames[i][:3, 3]))
        matrices.append(np.array(robot.axis_frames[i][:4, :4]))
    poses = np.array(poses)
    print("\npositions: \n")
    print(poses)
    print("\nfinal matrices: \n")
    print(matrices)
    print("\nTransitional matrices:\n")
    for i in range(robot.num_axis):
        print(np.array(robot.ts[i][:4, :4]))
        transitional_matrices.append(np.array(robot.ts[i][:4, :4]))

    layout = go.Layout(scene=dict(aspectmode="data"), height = max(10, int(height)), margin=dict(l=0, r=0, b=0, t=0))
    data=[go.Scatter3d(x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],mode='lines+markers', line=dict(color='grey', width=int(w_link)), marker=dict(color='black', size=int(w_joint)), showlegend = False)]
    for mat in matrices:
        data = data + drawMat(mat, int(w_arrow), float(l_arrow), float(c_arrow))
    fig = go.Figure(data=data, layout=layout)
    



    return fig, matrices, transitional_matrices

# layout
app.layout = html.Div([
    html.H1("Model your own robot using DH parameters"),
    html.H2("Settings"),
    html.Div([
        html.Div(["number of joints: ", dcc.Input(id='nbr_joints', value='6', type='number', style={'width': '5em'})], style={"fontWeight": "bold"}), html.Br(),
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
        dcc.Input(id='size_cone_arrow', value='0.1', type='number', style={'width': '5em'})

    ]),
    html.H2("DH parameters"),
    html.Table([], id='joints_table'),
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
        dcc.Input(id='rot-i', value=0, type='number', style={'width': '5em'}),
        " i + ",
        dcc.Input(id='rot-j', value=0, type='number', style={'width': '5em'}),
        " j + ",
        dcc.Input(id='rot-k', value=0, type='number', style={'width': '5em'}),
        " k + ",
        dcc.Input(id='rot-w', value=0, type='number', style={'width': '5em'}),
    ]),
    html.Br(),
    html.Div([
        html.Button('Calculate Inverse', id='inverse-btn', n_clicks=0, style={'fontSize': '1.1em'}),
        html.Button('Reset Angles', id='angle-reset-btn', n_clicks=0, style={'fontSize': '1.1em'}),
        html.Button('Reset DH parameters', id='dh-reset-btn', n_clicks=0, style={'fontSize': '1.1em'}),
    ]),
    dcc.Graph(id='graph-robot'),
    html.Div([
        html.H2("Transformation matrices"),
        html.Table([], id='mats_table', style={'fontSize':25})
    ])

], style={'marginLeft': "1rem", 'fontSize': 20})


# change robot parameters
# @app.callback(
#     Output(component_id='test-input', component_property='value'),
#     State(component_id='joints_table', component_property='children'),
#     Input(component_id={"name": ALL, "type": "dyn-in"}, component_property='value')
# )
# def update_robot(test_in, val):
#     print("hey" + str(val))
#     head = test_in[0]
#     content = test_in[1]
#     dh_params = []
#     print([e['props']['children'] for e in head['props']['children']['props']['children']])
#     for l in content['props']['children']:
#         axis = [float(e['props']['children']['props']['value']) for e in l['props']['children'][1:]]
#         print(axis)
#         dh_params.append([axis[0], axis[1], axis[2]/180*pi, axis[3]/180*pi])
#     print(dh_params)
#     robot = RobotSerial(dh_params)

#     return 5

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
    Input(component_id='size_cone_arrow', component_property='value')
)
def update_robot(params, angles, h, w_link, w_joint, w_arrow, l_arrow, c_arrow):
    print("update robot")
    global robot
    triggered_id = ctx.triggered_id
    try:
        if triggered_id['type'] == "dyn-in":
            dh_params = []
            for l in range(0, int(len(params)/4)):
                axis = [float(n) for n in params[4*l:4*l+4]]
                dh_params.append([axis[0], axis[1], axis[2]/180*pi, axis[3]/180*pi])
            dh_params = np.array(dh_params)
            print(dh_params)
            robot = RobotSerial(dh_params)
    except:
        pass
    robot_angles = np.array([float(a)/180*pi for a in angles])
    f = robot.forward(robot_angles)
    
    fig, matrices, matr_trans = draw_robot(robot, h, w_link, w_joint, w_arrow, l_arrow, c_arrow)


    # fig.update_layout(transition_duration=500)
    pos = f.t_3_1.reshape([3, ])
    quat = f.q_4

    return fig, \
        "{:.6f}".format(pos[0]), "{:.6f}".format(pos[1]), "{:.6f}".format(pos[2]), \
        "{:.6f}".format(quat[0]), "{:.6f}".format(quat[1]), "{:.6f}".format(quat[2]), "{:.6f}".format(quat[3]), \
        [html.Thead(
            html.Tr([html.Th("joint"), html.Th("Transition joint (i-1) to joint i"), html.Th("Transition up to joint i")])
        ),
        html.Tbody([
            html.Tr([
                    html.Th(i), 
                    html.Td(html.Div([
                        "{}".format(matr_trans[i-1][0]), html.Br(), 
                        "{}".format(matr_trans[i-1][1]), html.Br(),
                        "{}".format(matr_trans[i-1][2]), html.Br(),
                        "{}".format(matr_trans[i-1][3])
                        ])),
                    html.Td(html.Div([
                        "{}".format(matrices[i][0]), html.Br(), 
                        "{}".format(matrices[i][1]), html.Br(),
                        "{}".format(matrices[i][2]), html.Br(),
                        "{}".format(matrices[i][3])
                        ]))
                    ]) for i in range(1,len(matrices))]
        )]

# @app.callback(
#     Output('graph-robot', 'figure'),
#     Output('pos-x', 'value'),
#     Output('pos-y', 'value'),
#     Output('pos-z', 'value'),
#     Output('rot-i', 'value'),
#     Output('rot-j', 'value'),
#     Output('rot-k', 'value'),
#     Output('rot-w', 'value'),
#     Output('mats_table', 'children'),
#     Input(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
#     Input(component_id='fig_height', component_property='value'),
#     Input(component_id='width_link', component_property='value'),
#     Input(component_id='width_joint', component_property='value'),
#     Input(component_id='width_arrow', component_property='value'),
#     Input(component_id='length_arrow', component_property='value'),
#     Input(component_id='size_cone_arrow', component_property='value')
# )
# def update_robot_angles(angles, h, w_link, w_joint, w_arrow, l_arrow, c_arrow):
#     print("update robot angles")
#     global robot
#     robot_angles = np.array([float(a)/180*pi for a in angles])
#     f = robot.forward(robot_angles)

#     fig, matrices = draw_robot(robot, h, w_link, w_joint, w_arrow, l_arrow, c_arrow)



#     # fig.update_layout(transition_duration=500)
#     pos = f.t_3_1.reshape([3, ])
#     quat = f.q_4

#     return fig, \
#         "{:.6f}".format(pos[0]), "{:.6f}".format(pos[1]), "{:.6f}".format(pos[2]), \
#         "{:.6f}".format(quat[0]), "{:.6f}".format(quat[1]), "{:.6f}".format(quat[2]), "{:.6f}".format(quat[3]), \
#         [html.Thead(
#             html.Tr([html.Th("joint"), html.Th("Matrix")])
#         ),
#         html.Tbody([
#             html.Tr([
#                     html.Th(i), 
#                     html.Td(html.Div([
#                         "{}".format(matrices[i][0]), html.Br(), 
#                         "{}".format(matrices[i][1]), html.Br(),
#                         "{}".format(matrices[i][2]), html.Br(),
#                         "{}".format(matrices[i][3])
#                         ]))
#                     ]) for i in range(1,len(matrices))]
#         )]

@app.callback(
    Input('inverse-btn', 'n_clicks'),
    Input('angle-reset-btn', 'n_clicks'),
    State('pos-x', 'value'),
    State('pos-y', 'value'),
    State('pos-z', 'value'),
    State('rot-i', 'value'),
    State('rot-j', 'value'),
    State('rot-k', 'value'),
    State('rot-w', 'value'),
    Output(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
)
def calc_inverse(n_c_inv, n_c_res, x, y, z, i, j, k, w):
    triggered_id = ctx.triggered_id
    global robot

    if triggered_id == 'inverse-btn':
        print("calc inverse")

        xyz = np.array([[x], [y], [z]])
        quat = np.array([i, j, k, w])
        end = Frame.from_q_4(quat, xyz)
        robot.inverse(end)
        angles = robot.axis_values

        return tuple(["{:.3f}".format(a/pi*180) for a in angles])
    else:
        print("reset angles")

        angles = [0 for i in range(robot.num_axis)]

        return tuple(["{:.3f}".format(a/pi*180) for a in angles])


@app.callback(
    # State(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
    # State('nbr_joints', 'value'),
    Input('dh-reset-btn', 'n_clicks'),
    Output(component_id={"name": ALL, "type": "dyn-in"}, component_property='value'),
)
def reset_dh(n_c):
    print("\n\n\n\n\n\nWHYYYYY???????\n\n\n\n\n\n")
    triggered_id = ctx.triggered_id
    global robot
    print(triggered_id)
    return [0 for _ in range(robot.num_axis*4)]

# @app.callback(
#     Input('angle-reset-btn', 'n_clicks'),
#     Output(component_id={"name": ALL, "type": "dyn-in-angle"}, component_property='value'),
# )
# def reset_angles(n_c):
#     print("reset angles")
#     global robot

#     angles = [0 for i in range(robot.num_axis)]

#     return tuple(["{:.3f}".format(a/pi*180) for a in angles])

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
    print("params+angles")
    print(params)
    print(angles)
    return [
        html.Thead(
            html.Tr([html.Th("joint"), html.Th("d"), html.Th("a"), html.Th("alpha [°]"), html.Th("theta [°]"), html.Th("angle [°]")])
        ),
        html.Tbody([
            html.Tr([
                    html.Th(i), 
                    html.Td(dcc.Input(id={"name": 'd' + str(i), "type": "dyn-in"}, value=params[4*(i-1)] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'a' + str(i), "type": "dyn-in"}, value=params[4*(i-1)+1] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'alpha' + str(i), "type": "dyn-in"}, value=params[4*(i-1)+2] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'theta' + str(i), "type": "dyn-in"}, value=params[4*(i-1)+3] if 4*(i-1)+3 < len(params) else 0, type='number', style={'width': '5em'})), 
                    html.Td(dcc.Input(id={"name": 'angle' + str(i), "type": "dyn-in-angle"}, value=angles[(i-1)] if (i-1) < len(angles) else 0, type='number', style={'width': '5em'}))
                    ]) for i in range(1,max(2, int(joints)+1))
        ]),

    ], max(1, int(joints))


# # change figure height
# @app.callback(
#     Output('graph-robot', 'figure'),
#     State('graph-robot', 'figure'),
#     Input(component_id='fig_height', component_property='value'),
#     Input(component_id='width_link', component_property='value'),
#     Input(component_id='width_joint', component_property='value'),
#     Input(component_id='width_arrow', component_property='value'),
#     Input(component_id='length_arrow', component_property='value')
#     )
# def update_figure(fig_old, h):
#     print("update figure")
#     fig = go.Figure(fig_old)
#     return fig.update_layout(height = max(10, int(h)))

if __name__ == '__main__':
    app.run_server(debug=True)

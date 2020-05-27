import dash
from dash.dependencies import Output, Input
import dash_daq as daq
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
import plotly
import plotly.graph_objs as go
import sqlite3
from urllib.request import pathname2url

# Carrega os dados que foram inseridos no banco
def loadBuffer():
    # Le as ultimas 10 linhas do banco de dados
    c = conn.cursor()
    buffer = c.execute('SELECT * FROM '
                       '(SELECT rowid, * FROM monitor ORDER BY rowid DESC LIMIT 10) ORDER BY rowid ASC;').fetchall()
    c.close()

    # Salva em um vetor
    loaded_buffer = []
    for j in range(12):
        loaded_buffer.append([x[j] for x in buffer])

    return(loaded_buffer)

available_indicators = ['Phase 1', 'Phase 2', 'Circuit 1', 'Circuit 2', 'Circuit 3']
colors = ['#6200EE', '#3700B3', '#03DAC6', '#018786']

# Abre o arquivo do banco de dados para leitura
dburi = 'file:{}?mode=ro'.format(pathname2url("example.db"))
conn = sqlite3.connect(dburi, uri=True, check_same_thread=False)

# Define o tema que sera utilizado
theme1 = {
    'dark': True,
    'detail': '#007439',
    'primary': '#00EA64',
    'secondary': '#6E6E6E'
}

# Define a estrutura do HTML da pagina
body = dbc.Container([
    dbc.Row
        (dbc.Col(dcc.Dropdown(id='field', options=[{'label': i, 'value': i} for i in available_indicators],
                              value='Phase 1'), width="6" )),
    dbc.Row([
        dbc.Col(dcc.Graph(id='current_graph', animate=True)),
        dbc.Col(dcc.Graph(id='power_graph', animate=True))]),
    dbc.Row([
        dbc.Col(dcc.Graph(id='two_power_graph', animate=True)),
        dbc.Col(daq.Gauge(id='energy_gauge', label="Energy consumption", value=50, max=100, showCurrentValue=True, units="kWh")),
        dbc.Col(dcc.Graph(id='all_power_graph', animate=True))], align="center",),

    dcc.Interval(id='graph-update', interval=1000, n_intervals = 0)
])

# Inicializa o dashboard
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.layout = html.Div([body])


# Grafico de corrente
@app.callback(Output('current_graph', 'figure'),
              [Input('graph-update', 'n_intervals'),
               Input('field', 'value')])
def update_graph1_scatter(n, field_value):
    filtered_buffer = loadBuffer()
    k = 2*(available_indicators.index(field_value))+1
    X = filtered_buffer[0]
    Y1 = filtered_buffer[k]

    data = plotly.graph_objs.Scatter(x=list(X), y=list(Y1), name='Scatter', mode='lines+markers')
    return {'data': [data],
            'layout' : go.Layout(xaxis=dict(range=[min(X),max(X)]),
                                 yaxis=dict(range=[min(Y1), max(Y1)], title='CURRENT (A)'))}

# Grafico de potencia
@app.callback(Output('power_graph', 'figure'),
              [Input('graph-update', 'n_intervals'),
               Input('field', 'value')])
def update_graph2_scatter(n, field_value):
    filtered_buffer = loadBuffer()
    k = 2*(available_indicators.index(field_value))+2
    X = filtered_buffer[0]
    Y2 = filtered_buffer[k]

    data = plotly.graph_objs.Scatter(x=list(X), y=list(Y2), name='Scatter', mode='lines+markers')
    return {'data': [data],
            'layout' : go.Layout(xaxis=dict(range=[min(X),max(X)]),
                                 yaxis=dict(range=[min(Y2), max(Y2)], title='POWER (W)'))}

# Energia consumida
@app.callback(
    dash.dependencies.Output('energy_gauge', 'value'),
    [dash.dependencies.Input('graph-update', 'n_intervals')])
def update_output(n):
    filtered_buffer = loadBuffer()
    return filtered_buffer[11][9]

# Potencia por circuito
@app.callback(Output('all_power_graph', 'figure'),
              [Input('graph-update', 'n_intervals')])
def update_graph3_Pie(n):
    filtered_buffer = loadBuffer()
    Y3 = [0, 0, 0, 0]

    Y3[0] = filtered_buffer[6][9]
    Y3[1] = filtered_buffer[8][9]
    Y3[2] = filtered_buffer[10][9]
    Y3[3] = filtered_buffer[2][9] + filtered_buffer[4][9] - Y3[0] - Y3[1] - Y3[2]

    data = plotly.graph_objs.Pie(labels=['Circuit 1', 'Circuit 2', 'Circuit 3', 'Others'], values=list(Y3), marker = dict(colors=colors))
    return {'data': [data],
            'layout' : go.Layout(title='Power by circuit (W)')}

# Potencia por fase
@app.callback(Output('two_power_graph', 'figure'),
              [Input('graph-update', 'n_intervals')])
def update_graph4_Pie(n):
    filtered_buffer = loadBuffer()
    Y4 = [0, 0]

    Y4[0] = filtered_buffer[2][9]
    Y4[1] = filtered_buffer[4][9]

    data = plotly.graph_objs.Pie(labels=['Phase 1', 'Phase 2'], values=list(Y4), marker = dict(colors=colors))
    return {'data': [data],
            'layout' : go.Layout(title='Power by Phase (W)')}

if __name__ == '__main__':
    app.run_server(debug=True)


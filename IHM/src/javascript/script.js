//Definição de tags e labels:
const ctx1 = document.getElementById('real-time-chart1').getContext('2d');
const ctx2 = document.getElementById('real-time-chart2').getContext('2d');
const ctx3 = document.getElementById('real-time-chart3').getContext('2d');
const ctx4 = document.getElementById('real-time-chart4').getContext('2d');
const ctx5 = document.getElementById('real-time-chart5').getContext('2d');
const ctx6 = document.getElementById('real-time-chart6').getContext('2d');

//Definição de variáveis e constantes:
const ts = 10*10e-3;
var recording = false;

//Inicializa MQTT:
const options = {
    clientId: 'IHM',
    host: '192.168.137.1',
    port: 1883,
    protocol: 'mqtt'
};
const topics = ['reply', 'sampling1', 'sampling2', 'sampling3', 'sampling4'];
const client = new Paho.MQTT.Client(options.host,Number(options.port),options.clientId);
client.onConnectionLost = onConnectionLost;
client.onMessageArrived = onMessageArrived;
client.connect({
    onSuccess: onConnect
});

//Função de conexão perdida do MQTT:
function onConnectionLost(responseObject){
    client.connect({
        onSuccess: onConnect
    });
    console.log(responseObject);
}
//Função de mensagem recebida do MQTT:
function onMessageArrived(message){
    //console.log(message.destinationName + ": " + message.payloadString);
    if(message.destinationName == 'reply')
    {
        validCOM(message.payloadString);
    }
    if(message.destinationName == 'sampling1')
    {
        sampHandler1(message.payloadString);
    }
    if(message.destinationName == 'sampling2')
    {
        sampHandler2(message.payloadString);
    } 
    if(message.destinationName == 'sampling3')
    {
        sampHandler3(message.payloadString);
    }
    if(message.destinationName == 'sampling4')
    {
        //sampHandler4(message.payloadString);
    }
}
//Função 'ao conectar' do MQTT:
function onConnect(){
    console.log("Conectado ao servidor MQTT");
    
    for (let i = 0; i < topics.length; i++) {
        client.subscribe(topics[i]);
    }

    tempStatus('Successful server conection !');
    statusmsg = 'Waiting for controller conection...';
}
//Função para enviar mensagem no tópico do MQTT:
function sendMqtt(topic, message) {
    client.send(topic, message);
}

//Conversão de unidades:
function degrees(rad)
{
    return (rad*180.0/Math.PI);
}
function radians(deg)
{
    return (deg*Math.PI/180.0);
}

//Função para cinemática direta do manipulador:
function forwardKinematics(angles) {
    let x = (-Math.cos(radians(angles[0]))*((230*Math.sin(radians(angles[1])))+(230*Math.sin(radians(angles[1]+angles[2])))+(124.5*Math.sin(radians(angles[1]+angles[2]+angles[3])))));
    let y = (-Math.sin(radians(angles[0]))*((230*Math.sin(radians(angles[1])))+(230*Math.sin(radians(angles[1]+angles[2])))+(124.5*Math.sin(radians(angles[1]+angles[2]+angles[3])))));
    let z = (187+(230*Math.cos(radians(angles[1])))+(230*Math.cos(radians(angles[1]+angles[2])))+(124.5*Math.cos(radians(angles[1]+angles[2]+angles[3]))));
    let se = (-Math.sin(radians(angles[1]+angles[2]+angles[3])));
    let ce = Math.cos(radians(angles[1]+angles[2]+angles[3]));
    let a = 0;
    let e = degrees(Math.atan2(se,ce));
    let r = 0;
    if(Math.abs(Math.sin(radians(e)))>=1e-6)
    {
        a = angles[0];
        let sr = (-Math.sin(radians(angles[4]))*Math.sin(radians(angles[1]+angles[2]+angles[3]))/Math.sin(radians(e)));
        let cr = (-Math.cos(radians(angles[4]))*Math.sin(radians(angles[1]+angles[2]+angles[3]))/Math.sin(radians(e)));
        r = degrees(Math.atan2(sr,cr));
    }else
    {
        a = 0;
        r = (angles[0]+(angles[4]*Math.cos(radians(angles[1]+angles[2]+angles[3]))));
        if(r>180)
        {
            r = (180-r);
        }            
        if(r< -180)
        {
            r = (r+180);
        }            
    }                
    return [Math.round(x, 1), Math.round(y, 1), Math.round(z, 1), Math.round(a, 1), Math.round(e, 1), Math.round(r, 1)];
}


var current_time = 0;

//Função para ajustar os eixos dinamicamente:
function dynGraph() {
    if(chartData.time.length > (20/ts))
    {
        for (let i=0; i<6; i++) 
        {
            chartData.data[i].shift();
            chartData.ref[i].shift();
        }
        chartData.time.shift();
    }
    var min = parseFloat(parseFloat(chartData.time[0]).toFixed(1));
    var max = parseFloat(parseFloat(chartData.time[chartData.time.length-1]).toFixed(1));
    chart1.options.scales.x.min = min;
    chart1.options.scales.x.max = max;
    chart2.options.scales.x.min = min;
    chart2.options.scales.x.max = max;
    chart3.options.scales.x.min = min;
    chart3.options.scales.x.max = max;
    chart4.options.scales.x.min = min;
    chart4.options.scales.x.max = max;
    chart5.options.scales.x.min = min;
    chart5.options.scales.x.max = max;
    chart6.options.scales.x.min = min;
    chart6.options.scales.x.max = max;

    chart1.update();
    chart2.update();
    chart3.update();
    chart4.update();
    chart5.update();
    chart6.update();
}

//Função para incorporar a amostragem das juntas:
function sampHandler1(payload) {
    var vals = payload.split(',');
    if(recording)
    {
        recordingData.time.push(current_time.toFixed(2));
        for(let i=0; i<6; i++) 
        {
            recordingData.dof[i].push(vals[i]);
        }
    }
    let cart = forwardKinematics([parseFloat(vals[0]), parseFloat(vals[1]), parseFloat(vals[2]), parseFloat(vals[3]), parseFloat(vals[4])]);
    updateJVals(parseFloat(vals[0]), parseFloat(vals[1]), parseFloat(vals[2]), parseFloat(vals[3]), parseFloat(vals[4]), parseFloat(vals[5]));
    updateCVals(cart[0],cart[1],cart[2],cart[3],cart[4],cart[5]);
    if(graphtype.value == 'A')
    {
        if(gdlselect.value == 'J1')
        {
            chartData.data[0].push(vals[0]);
        }
        if(gdlselect.value == 'J2')
        {
            chartData.data[0].push(vals[1]);
        }
        if(gdlselect.value == 'J3')
        {
            chartData.data[0].push(vals[2]);
        }
        if(gdlselect.value == 'J4')
        {
            chartData.data[0].push(vals[3]);
        }
        if(gdlselect.value == 'J5')
        {
            chartData.data[0].push(vals[4]);
        }
        if(gdlselect.value == 'J6')
        {
            chartData.data[0].push(vals[5]);
        }
    }
    if(graphtype.value == 'B')
    {
        for (let i=0; i<6; i++) 
        {
            chartData.data[i].push(vals[i]);
        }
    }
    if(graphtype.value == 'E')
    {
        for (let i=0; i<6; i++) 
        {
            chartData.data[i].push(cart[i]);
        }
    }
    chartData.time.push(current_time.toFixed(2));
    dynGraph();
    current_time += ts;  
}

//Função para incorporar a referência das juntas:
function sampHandler2(payload) {
    var vals = payload.split(',');
    if(recording)
    {
        for(let i=0; i<6; i++) 
        {
            recordingData.ref[i].push(vals[i]);
        }
    }
    if(graphtype.value == 'A')
    {
        if(gdlselect.value == 'J1')
        {
            chartData.ref[0].push(vals[0]);
        }
        if(gdlselect.value == 'J2')
        {
            chartData.ref[0].push(vals[1]);
        }
        if(gdlselect.value == 'J3')
        {
            chartData.ref[0].push(vals[2]);
        }
        if(gdlselect.value == 'J4')
        {
            chartData.ref[0].push(vals[3]);
        }
        if(gdlselect.value == 'J5')
        {
            chartData.ref[0].push(vals[4]);
        }
        if(gdlselect.value == 'J6')
        {
            chartData.ref[0].push(vals[5]);
        }
    }
    if(graphtype.value == 'B')
    {
        for (let i=0; i<6; i++) 
        {
            chartData.ref[i].push(vals[i]);
        }
    }
    if(graphtype.value == 'E')
    {
        let cart = forwardKinematics([parseFloat(vals[0]), parseFloat(vals[1]), parseFloat(vals[2]), parseFloat(vals[3]), parseFloat(vals[4])]);
        for (let i=0; i<6; i++) 
        {
            chartData.ref[i].push(cart[i]);
        }
    }
    dynGraph();
}

//Função para incorporar o sinal de controle das juntas:
function sampHandler3(payload) {
    var vals = payload.split(',');
    if(recording)
    {
        for(let i=0; i<6; i++) 
        {
            recordingData.u[i].push(vals[i]);
        }
    }
    if(graphtype.value == 'A')
    {
        if(gdlselect.value == 'J1')
        {
            chartData.data[1].push(vals[0]);
        }
        if(gdlselect.value == 'J2')
        {
            chartData.data[1].push(vals[1]);
        }
        if(gdlselect.value == 'J3')
        {
            chartData.data[1].push(vals[2]);
        }
        if(gdlselect.value == 'J4')
        {
            chartData.data[1].push(vals[3]);
        }
        if(gdlselect.value == 'J5')
        {
            chartData.data[1].push(vals[4]);
        }
        if(gdlselect.value == 'J6')
        {
            chartData.data[1].push(vals[5]);
        }
    }
    if(graphtype.value == 'C')
    {
        for (let i=0; i<6; i++) 
        {
            chartData.data[i].push(vals[i]);
        }
    }
    dynGraph();
}







const tabs = document.querySelectorAll('.tab-btn');

tabs.forEach(tab => tab.addEventListener('click', () => tabClicked(tab)))

const tabClicked = (tab) => {
    tabs.forEach(tab => tab.classList.remove('active'));
    tab.classList.add('active');

    const contents = document.querySelectorAll('.content');
    contents.forEach(content => content.classList.remove('show'));

    const contentId = tab.getAttribute('content-id');
    const content = document.getElementById(contentId);
    
    content.classList.add('show');
}

const currentActiveTab = document.querySelector('.tab-btn.active');
tabClicked(currentActiveTab);



function updateLineNumbers() {
    var textarea = document.getElementById('editor');
    var lineNumbers = document.getElementById('line-numbers');
    
    // Dividir o conteúdo do textarea em linhas
    var lines = textarea.value.split('\n');
    
    var numbers = '';
    for (var i = 1; i <= lines.length; i++) {
        numbers += i + '\n';
    }

    // Atualizar a visualização da numeração das linhas
    lineNumbers.textContent = numbers;
}

const textarea = document.getElementById('editor');
textarea.addEventListener('input', updateLineNumbers);

// Chama a função para definir as linhas ao carregar a página
window.onload = function() {
    updateLineNumbers();
};

const startButton = document.querySelector('.start-btn');
const holdButton = document.querySelector('.hold-btn');
startButton.addEventListener('click', () => {
    startButton.style.display = "none";
    holdButton.style.display = "flex";
    sendMqtt('request', 'S');
    tempStatus('Start moving!');
    //current_time = 0;
});
holdButton.addEventListener('click', () => {
    holdButton.style.display = "none";
    startButton.style.display = "flex";
    sendMqtt('request', 'H');
    tempStatus('Hold moving!');
});

//Função para salvar os dados em TXT:
function saveDataToTXT() {
    var txtContent = "t\tj1\tj2\tj3\tj4\tj5\tgrip\tref1\tref2\tref3\tref4\tref5\trefg\tu1\tu2\tu3\tu4\tu5\tu6\n";
    for(let j=0; j<recordingData.time.length; j++) 
    {
        var line = recordingData.time[j];
        for(let i=0; i<6; i++) 
        {
            line += '\t'+recordingData.dof[i][j];
        }
        for(let i=0; i<6; i++) 
        {
            line += '\t'+recordingData.ref[i][j];
        }
        for(let i=0; i<6; i++) 
        {
            line += '\t'+recordingData.u[i][j];
        }
        line += '\n';
        txtContent += line;
    }

    const blob = new Blob([txtContent], { type: 'text/txt' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = 'data.txt';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);

    recordingData.time.length = 0;
    for(let i=0; i<6; i++) 
    {
        recordingData.dof[i].length = 0;
        recordingData.ref[i].length = 0;
        recordingData.u[i].length = 0;
    }
}

const recButton = document.querySelector('.rec-btn');
const stopButton = document.querySelector('.stop-btn');
recButton.addEventListener('click', () => {
    recButton.style.display = "none";
    stopButton.style.display = "flex";
    current_time = 0;
    recording = true;
    tempStatus('Recording!');
});
stopButton.addEventListener('click', () => {
    stopButton.style.display = "none";
    recButton.style.display = "flex";
    recording = false;
    tempStatus('Saving Data!');
    saveDataToTXT();
});

var statusmsg = "Waiting connection...";

function tempStatus(message) {
    const statusMessage = document.getElementById('status-message');
    statusMessage.textContent = message;

    // Esconder a mensagem após 5 segundos (5000 milissegundos)
    setTimeout(() => {
        statusMessage.textContent = statusmsg;
    }, 5000); // Tempo em milissegundos
}

function setStatus(message) {
    const statusMessage = document.getElementById('status-message');
    statusMessage.textContent = message;

    statusmsg = message;
}

//Estrutura para salvamento dos dados:
var recordingData = {
    time: [],
    dof: [[], [], [], [], [], []],
    ref: [[], [], [], [], [], []],
    u: [[], [], [], [], [], []],
};


//Dados para os gráficos:
var chartData = {
    time: [],
    data: [[], [], [], [], [], []],
    ref: [[], [], [], [], [], []],
};

//Color vector:
const colors = ['rgba(100, 100, 100, 1)', 'rgba(255, 0, 0, 1)', 'rgba(0, 0, 255, 1)', 'rgba(0, 255, 0, 1)', 'rgba(255, 255, 0, 1)', 'rgba(255, 0, 255, 1)', 'rgba(0, 255, 255, 1)'];

const chart1 = new Chart(ctx1, {
    type: 'line',
    data: {
        labels: chartData.time,
        datasets: [{
            label: 'θ₁(t)',
            data: chartData.data[0],
            borderColor: colors[1],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
        },
        {
            label: 'θref₁(t)',
            data: chartData.ref[0],
            borderColor: colors[0],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
            borderDash: [5, 5],
        },
    ]},
    options: {
        responsive: true,
        animation: false,
        maintainAspectRatio: false,
        color: '#000',
        layout: {
            padding: {
                left: 0,
            },
        },
        plugins: {
            title: {
                display: true,
                text: 'Joint 1 Value',
                color: '#000',
            },
            legend: {
                display: true,
            },
        },
        scales: {
            x: {
                type: 'linear',
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tempo [s]',
                    color: '#000',
                },
                ticks: {
                    stepSize: 1,
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
            y: {
                beginAtZero: false,
                min: -170,
                max: 170,
                title: {
                    display: true,
                    text: 'Angle [°]',
                    color: '#000',
                },
                ticks: {
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
        },
    },
});

const chart2 = new Chart(ctx2, {
    type: 'line',
    data: {
        labels: chartData.time,
        datasets: [{
            label: 'u₂(t)',
            data: chartData.data[1],
            borderColor: colors[2],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
        },
        {
            label: '',
            data: chartData.ref[1],
            borderColor: colors[0],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
            borderDash: [5, 5],
        },
    ]},
    options: {
        responsive: true,
        animation: false,
        maintainAspectRatio: false,
        color: '#000',
        layout: {
            padding: {
                left: 0,
            },
        },
        plugins: {
            title: {
                display: true,
                text: 'Control Signal',
                color: '#000',
            },
            legend: {
                display: true,
            },
        },
        scales: {
            x: {
                type: 'linear',
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tempo [s]',
                    color: '#000',
                },
                ticks: {
                    stepSize: 1,
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
            y: {
                beginAtZero: false,
                min: -100,
                max: 100,
                title: {
                    display: true,
                    text: 'Duty Cycle [%]',
                    color: '#000',
                },
                ticks: {
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
        },
    },
});

const chart3 = new Chart(ctx3, {
    type: 'line',
    data: {
        labels: chartData.time,
        datasets: [{
            label: 'θ₃(t)',
            data: chartData.data[2],
            borderColor: colors[3],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
        },
        {
            label: 'θref₃(t)',
            data: chartData.ref[2],
            borderColor: colors[0],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
            borderDash: [5, 5],
        },
    ]},
    options: {
        responsive: true,
        animation: false,
        maintainAspectRatio: false,
        color: '#000',
        layout: {
            padding: {
                left: 0,
            },
        },
        plugins: {
            title: {
                display: true,
                text: 'Joint 3 Value',
                color: '#000',
            },
            legend: {
                display: true,
            },
        },
        scales: {
            x: {
                type: 'linear',
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tempo [s]',
                    color: '#000',
                },
                ticks: {
                    stepSize: 1,
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
            y: {
                beginAtZero: false,
                title: {
                    display: true,
                    text: 'Angle [°]',
                    color: '#000',
                },
                ticks: {
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
        },
    },
});

const chart4 = new Chart(ctx4, {
    type: 'line',
    data: {
        labels: chartData.time,
        datasets: [{
            label: 'θ₄(t)',
            data: chartData.data[3],
            borderColor: colors[4],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
        },
        {
            label: 'θref₄(t)',
            data: chartData.ref[3],
            borderColor: colors[0],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
            borderDash: [5, 5],
        },
    ]},
    options: {
        responsive: true,
        animation: false,
        maintainAspectRatio: false,
        color: '#000',
        layout: {
            padding: {
                left: 0,
            },
        },
        plugins: {
            title: {
                display: true,
                text: 'Joint 4 Value',
                color: '#000',
            },
            legend: {
                display: true,
            },
        },
        scales: {
            x: {
                type: 'linear',
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tempo [s]',
                    color: '#000',
                },
                ticks: {
                    stepSize: 1,
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
            y: {
                beginAtZero: false,
                title: {
                    display: true,
                    text: 'Angle [°]',
                    color: '#000',
                },
                ticks: {
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
        },
    },
});

const chart5 = new Chart(ctx5, {
    type: 'line',
    data: {
        labels: chartData.time,
        datasets: [{
            label: 'θ₅(t)',
            data: chartData.data[4],
            borderColor: colors[5],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
        },
        {
            label: 'θref₅(t)',
            data: chartData.ref[4],
            borderColor: colors[0],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
            borderDash: [5, 5],
        },
    ]},
    options: {
        responsive: true,
        animation: false,
        maintainAspectRatio: false,
        color: '#000',
        layout: {
            padding: {
                left: 0,
            },
        },
        plugins: {
            title: {
                display: true,
                text: 'Joint 5 Value',
                color: '#000',
            },
            legend: {
                display: true,
            },
        },
        scales: {
            x: {
                type: 'linear',
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tempo [s]',
                    color: '#000',
                },
                ticks: {
                    stepSize: 1,
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
            y: {
                beginAtZero: false,
                title: {
                    display: true,
                    text: 'Angle [°]',
                    color: '#000',
                },
                ticks: {
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
        },
    },
});

const chart6 = new Chart(ctx6, {
    type: 'line',
    data: {
        labels: chartData.time,
        datasets: [{
            label: 'θ₆(t)',
            data: chartData.data[5],
            borderColor: colors[6],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
        },
        {
            label: 'θref₆(t)',
            data: chartData.ref[5],
            borderColor: colors[0],
            borderWidth: 2,
            fill: false,
            pointRadius: 0,
            borderDash: [5, 5],
        },
    ]},
    options: {
        responsive: true,
        animation: false,
        maintainAspectRatio: false,
        color: '#000',
        layout: {
            padding: {
                left: 0,
            },
        },
        plugins: {
            title: {
                display: true,
                text: 'Joint 6 Value',
                color: '#000',
            },
            legend: {
                display: true,
            },
        },
        scales: {
            x: {
                type: 'linear',
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tempo [s]',
                    color: '#000',
                },
                ticks: {
                    stepSize: 1,
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
            y: {
                beginAtZero: false,
                title: {
                    display: true,
                    text: 'Angle [°]',
                    color: '#000',
                },
                ticks: {
                    color: '#000',
                },
                grid: {
                    color: '#dcdedc',
                },
            },
        },
    },
});



const coordtype = document.getElementById('coord-type');
const motionJoint = document.querySelector('.motion-joint');
const motionBase = document.querySelector('.motion-base');
const motionTool = document.querySelector('.motion-tool');
const motionTarget = document.querySelector('.motion-target');

//Configuração de tipo de coordenada:
coordtype.addEventListener('change', () => {
    if(coordtype.value == 'J')
    {
        motionJoint.style.display = 'block';
        motionBase.style.display = 'none';
        motionTool.style.display = 'none';
        motionTarget.style.display = 'none';
    }
    if(coordtype.value == 'B')
    {
        motionJoint.style.display = 'none';
        motionBase.style.display = 'block';
        motionTool.style.display = 'none';
        motionTarget.style.display = 'none';
    }
    if(coordtype.value == 'T')
    {
        motionJoint.style.display = 'none';
        motionBase.style.display = 'none';
        motionTool.style.display = 'block';
        motionTarget.style.display = 'none';
    }
    if(coordtype.value == 'G')
    {
        motionJoint.style.display = 'none';
        motionBase.style.display = 'none';
        motionTool.style.display = 'none';
        motionTarget.style.display = 'block';
    }     
});

const movetype = document.getElementById('move-type');
const framechoose = document.getElementById('frame-choose');
const tg1 = document.getElementById('tg1');
const tg2 = document.getElementById('tg2');
const tg3 = document.getElementById('tg3');
const tg4 = document.getElementById('tg4');
const tg5 = document.getElementById('tg5');
movetype.addEventListener('change', () => {
    if(movetype.value == 'MJ')
    {
        tg1.style.display = 'flex';
        tg1.placeholder = 'J1';
        tg2.style.display = 'flex';
        tg2.placeholder = 'J2';
        tg3.style.display = 'flex';
        tg3.placeholder = 'J3';
        tg4.style.display = 'flex';
        tg4.placeholder = 'J4';
        tg5.style.display = 'flex';
        tg5.placeholder = 'J5';
        framechoose.style.display = 'none';
    }
    if(movetype.value == 'ML')
    {
        tg1.style.display = 'flex';
        tg1.placeholder = 'X';
        tg2.style.display = 'flex';
        tg2.placeholder = 'Y';
        tg3.style.display = 'flex';
        tg3.placeholder = 'Z';
        tg4.style.display = 'flex';
        tg4.placeholder = 'e';
        tg5.style.display = 'flex';
        tg5.placeholder = 'r';
        framechoose.style.display = 'none';
    }
    if(movetype.value == 'MA')
    {
        tg1.style.display = 'flex';
        tg1.placeholder = 'X';
        tg2.style.display = 'flex';
        tg2.placeholder = 'Y';
        tg3.style.display = 'flex';
        tg3.placeholder = 'Z';
        tg4.style.display = 'none';
        tg5.style.display = 'none';
        framechoose.style.display = 'flex';
    }
});


const showj1 = document.getElementById('j1');
const showj2 = document.getElementById('j2');
const showj3 = document.getElementById('j3');
const showj4 = document.getElementById('j4');
const showj5 = document.getElementById('j5');
const showgrip = document.getElementById('grip');
const showj1m = document.getElementById('j1m');
const showj2m = document.getElementById('j2m');
const showj3m = document.getElementById('j3m');
const showj4m = document.getElementById('j4m');
const showj5m = document.getElementById('j5m');
const showgripm = document.getElementById('gripm');
const showj1h = document.getElementById('j1h');
const showj2h = document.getElementById('j2h');
const showj3h = document.getElementById('j3h');
const showj4h = document.getElementById('j4h');
const showj5h = document.getElementById('j5h');
const showgriph = document.getElementById('griph');
const showj1g = document.getElementById('j1g');
const showj2g = document.getElementById('j2g');
const showj3g = document.getElementById('j3g');
const showj4g = document.getElementById('j4g');
const showj5g = document.getElementById('j5g');
const showgripg = document.getElementById('gripg');
function updateJVals(j1, j2, j3, j4, j5, grip) {
    showj1.innerHTML = 'J1: '+j1.toFixed(1)+' °';
    showj2.innerHTML = 'J2: '+j2.toFixed(1)+' °';
    showj3.innerHTML = 'J3: '+j3.toFixed(1)+' °';
    showj4.innerHTML = 'J4: '+j4.toFixed(1)+' °';
    showj5.innerHTML = 'J5: '+j5.toFixed(1)+' °';
    showgrip.innerHTML = 'Grip: '+grip.toFixed(0)+' mm';
    showj1m.innerHTML = 'J1: '+j1.toFixed(1)+' °';
    showj2m.innerHTML = 'J2: '+j2.toFixed(1)+' °';
    showj3m.innerHTML = 'J3: '+j3.toFixed(1)+' °';
    showj4m.innerHTML = 'J4: '+j4.toFixed(1)+' °';
    showj5m.innerHTML = 'J5: '+j5.toFixed(1)+' °';
    showgripm.innerHTML = 'Grip: '+grip.toFixed(0)+' mm';
    showj1h.innerHTML = 'J1: '+j1.toFixed(1)+' °';
    showj2h.innerHTML = 'J2: '+j2.toFixed(1)+' °';
    showj3h.innerHTML = 'J3: '+j3.toFixed(1)+' °';
    showj4h.innerHTML = 'J4: '+j4.toFixed(1)+' °';
    showj5h.innerHTML = 'J5: '+j5.toFixed(1)+' °';
    showgriph.innerHTML = 'Grip: '+grip.toFixed(0)+' mm';
    showj1g.innerHTML = 'J1: '+j1.toFixed(1)+' °';
    showj2g.innerHTML = 'J2: '+j2.toFixed(1)+' °';
    showj3g.innerHTML = 'J3: '+j3.toFixed(1)+' °';
    showj4g.innerHTML = 'J4: '+j4.toFixed(1)+' °';
    showj5g.innerHTML = 'J5: '+j5.toFixed(1)+' °';
    showgripg.innerHTML = 'Grip: '+grip.toFixed(0)+' mm';
};

const showxb = document.getElementById('Xb');
const showyb = document.getElementById('Yb');
const showzb = document.getElementById('Zb');
const showab = document.getElementById('ab');
const showeb = document.getElementById('eb');
const showrb = document.getElementById('rb');
const showxt = document.getElementById('Xt');
const showyt = document.getElementById('Yt');
const showzt = document.getElementById('Zt');
const showat = document.getElementById('at');
const showet = document.getElementById('et');
const showrt = document.getElementById('rt');
const showxh = document.getElementById('Xh');
const showyh = document.getElementById('Yh');
const showzh = document.getElementById('Zh');
const showah = document.getElementById('ah');
const showeh = document.getElementById('eh');
const showrh = document.getElementById('rh');
const showxg = document.getElementById('Xg');
const showyg = document.getElementById('Yg');
const showzg = document.getElementById('Zg');
const showag = document.getElementById('ag');
const showeg = document.getElementById('eg');
const showrg = document.getElementById('rg');
function updateCVals(X, Y, Z, a, e, r) {
    showxb.innerHTML = 'X: '+X.toFixed(0)+' mm';
    showyb.innerHTML = 'Y: '+Y.toFixed(0)+' mm';
    showzb.innerHTML = 'Z: '+Z.toFixed(0)+' mm';
    showab.innerHTML = 'a: '+a.toFixed(1)+' °';
    showeb.innerHTML = 'e: '+e.toFixed(1)+' °';
    showrb.innerHTML = 'r: '+r.toFixed(1)+' °';
    showxt.innerHTML = 'X: '+X.toFixed(0)+' mm';
    showyt.innerHTML = 'Y: '+Y.toFixed(0)+' mm';
    showzt.innerHTML = 'Z: '+Z.toFixed(0)+' mm';
    showat.innerHTML = 'a: '+a.toFixed(1)+' °';
    showet.innerHTML = 'e: '+e.toFixed(1)+' °';
    showrt.innerHTML = 'r: '+r.toFixed(1)+' °';
    showxh.innerHTML = 'X: '+X.toFixed(0)+' mm';
    showyh.innerHTML = 'Y: '+Y.toFixed(0)+' mm';
    showzh.innerHTML = 'Z: '+Z.toFixed(0)+' mm';
    showah.innerHTML = 'a: '+a.toFixed(1)+' °';
    showeh.innerHTML = 'e: '+e.toFixed(1)+' °';
    showrh.innerHTML = 'r: '+r.toFixed(1)+' °';
    showxg.innerHTML = 'X: '+X.toFixed(0)+' mm';
    showyg.innerHTML = 'Y: '+Y.toFixed(0)+' mm';
    showzg.innerHTML = 'Z: '+Z.toFixed(0)+' mm';
    showag.innerHTML = 'a: '+a.toFixed(1)+' °';
    showeg.innerHTML = 'e: '+e.toFixed(1)+' °';
    showrg.innerHTML = 'r: '+r.toFixed(1)+' °';
};

const newBtn = document.getElementById('new-btn');
newBtn.addEventListener('click', () => {
    if(!Boolean(textarea.value.length))
    {
        textarea.value = '';
        updateLineNumbers();
    }else
    {
        if(confirm('The current program will not be saved!'))
        {
            textarea.value = '';
            updateLineNumbers();
        }
    } 
});

const openBtn = document.getElementById('open-btn');
const fileinput = document.getElementById('file-input');
openBtn.addEventListener('click', () => {
    if(!Boolean(textarea.value.length))
    {
        fileinput.click();
    }else
    {
        if(confirm('The current program will not be saved!'))
        {
            fileinput.click();
        }
    }    
});

fileinput.addEventListener('change', () => {
    const file = fileinput.files[0];
    const reader = new FileReader();

    reader.onload = () => {
        textarea.value = reader.result;
        updateLineNumbers();
    };

    reader.readAsText(file);
    updateLineNumbers();
});

const saveBtn = document.getElementById('save-btn');
saveBtn.addEventListener('click', () => {
    if(Boolean(textarea.value.length))
    {
        const txtContent = textarea.value;
        const blob = new Blob([txtContent], { type: 'text/txt' });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = 'program.ed';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
    }else
    {
        alert('There is no script to save!');
    }    
});

const looptype = document.getElementById('loop-type');
const jointselect = document.getElementById('joint-select');
const constantes = document.getElementById('constantes');
const reference = document.getElementById('reference');

const constlabel = document.getElementById('constlabel');
const kpin = document.getElementById('kpin');
const kiin = document.getElementById('kiin');
const kdin = document.getElementById('kdin');
const botup = document.getElementById('botup');
const reflabel = document.getElementById('reflabel');
const refval = document.getElementById('refval');
const sendref = document.getElementById('sendref');

botup.addEventListener('click', () => {
    sendMqtt('request', 'C'+jointselect.value[1]+','+kpin.value.toString()+','+kiin.value.toString()+','+kdin.value.toString()+',');
});

looptype.addEventListener('change', () => {
    if(looptype.value == 'O')
    {
        constantes.style.display = 'none';
        reference.style.display = 'block';
        reflabel.innerHTML = 'PWM: - %';
        refval.placeholder = 'PWM';
    }
    if(looptype.value == 'C')
    {
        constantes.style.display = 'block';
        reference.style.display = 'block';
        reflabel.innerHTML = 'Setpoint: - °';
        refval.placeholder = 'SP';
    }    
});

const rtc1 = document.getElementById('rtc1');
const rtc2 = document.getElementById('rtc2');
const rtc3 = document.getElementById('rtc3');
const rtc4 = document.getElementById('rtc4');
const rtc5 = document.getElementById('rtc5');
const rtc6 = document.getElementById('rtc6');
const graphics = document.getElementById('graphics');

function singleLayout() {
    graphics.style.display = 'block';

    rtc1.style.display ='flex';
    rtc2.style.display ='flex';
    rtc3.style.display ='none';
    rtc4.style.display ='none';
    rtc5.style.display ='none';
    rtc6.style.display ='none';

    rtc1.style.width = '80vw';
    rtc2.style.width = '80vw';
};

function allLayout() {
    graphics.style.display = 'grid';

    rtc1.style.display ='flex';
    rtc2.style.display ='flex';
    rtc3.style.display ='flex';
    rtc4.style.display ='flex';
    rtc5.style.display ='flex';
    rtc6.style.display ='flex';

    rtc1.style.width = '30vw';
    rtc2.style.width = '30vw';
};

const graphtype = document.getElementById('graph-type');
const gdlselect = document.getElementById('gdl-select');

function updateView() {
    if(graphtype.value == 'A')
    {
        gdlselect.style.display = 'flex';
        singleLayout();
        if(gdlselect.value == 'J1')
        {
            chart1.options.plugins.title.text = 'Joint 1 Value';
            chart1.data.datasets[0].label = 'θ₁(t)';
            chart1.data.datasets[1].label = 'θref₁(t)';
            chart1.options.scales.y.title.text = 'Angle [°]';
            chart1.options.scales.y.min = -170;
            chart1.options.scales.y.max = 170;
            chart2.options.plugins.title.text = 'Control Signal 1';
            chart2.data.datasets[0].label = 'u₁(t)';
            chart2.data.datasets[1].label = '';
            chart2.options.scales.y.title.text = 'Duty Cycle [%]';  
            chart2.options.scales.y.min = -100;
            chart2.options.scales.y.max = 100;        
        }
        if(gdlselect.value == 'J2')
        {
            chart1.options.plugins.title.text = 'Joint 2 Value';
            chart1.data.datasets[0].label = 'θ₂(t)';
            chart1.data.datasets[1].label = 'θref₂(t)';
            chart1.options.scales.y.title.text = 'Angle [°]';
            chart1.options.scales.y.min = -90;
            chart1.options.scales.y.max = 30;
            chart2.options.plugins.title.text = 'Control Signal 2';
            chart2.data.datasets[0].label = 'u₂(t)';
            chart2.data.datasets[1].label = '';
            chart2.options.scales.y.title.text = 'Duty Cycle [%]';      
            chart2.options.scales.y.min = -100;
            chart2.options.scales.y.max = 100;   
        }
        if(gdlselect.value == 'J3')
        {
            chart1.options.plugins.title.text = 'Joint 3 Value';
            chart1.data.datasets[0].label = 'θ₃(t)';
            chart1.data.datasets[1].label = 'θref₃(t)';
            chart1.options.scales.y.title.text = 'Angle [°]';
            chart1.options.scales.y.min = -135;
            chart1.options.scales.y.max = 0;
            chart2.options.plugins.title.text = 'Control Signal 3';
            chart2.data.datasets[0].label = 'u₃(t)';
            chart2.data.datasets[1].label = '';
            chart2.options.scales.y.title.text = 'Duty Cycle [%]'; 
            chart2.options.scales.y.min = -100;
            chart2.options.scales.y.max = 100;         
        }
        if(gdlselect.value == 'J4')
        {
            chart1.options.plugins.title.text = 'Joint 4 Value';
            chart1.data.datasets[0].label = 'θ₄(t)';
            chart1.data.datasets[1].label = 'θref₄(t)';
            chart1.options.scales.y.title.text = 'Angle [°]';
            chart1.options.scales.y.min = -135;
            chart1.options.scales.y.max = 135;
            chart2.options.plugins.title.text = 'Control Signal 4';
            chart2.data.datasets[0].label = 'u₄(t)';
            chart2.data.datasets[1].label = '';
            chart2.options.scales.y.title.text = 'Duty Cycle [%]';
            chart2.options.scales.y.min = -100;
            chart2.options.scales.y.max = 100;          
        }
        if(gdlselect.value == 'J5')
        {
            chart1.options.plugins.title.text = 'Joint 5 Value';
            chart1.data.datasets[0].label = 'θ₅(t)';
            chart1.data.datasets[1].label = 'θref₅(t)';
            chart1.options.scales.y.title.text = 'Angle [°]';
            chart1.options.scales.y.min = -170;
            chart1.options.scales.y.max = 170;
            chart2.options.plugins.title.text = 'Control Signal 5';
            chart2.data.datasets[0].label = 'u₅(t)';
            chart2.data.datasets[1].label = '';
            chart2.options.scales.y.title.text = 'Duty Cycle [%]'; 
            chart2.options.scales.y.min = -100;
            chart2.options.scales.y.max = 100;         
        }
        if(gdlselect.value == 'J6')
        {
            chart1.options.plugins.title.text = 'Grip Opening';
            chart1.data.datasets[0].label = 'd(t)';
            chart1.data.datasets[1].label = 'dref(t)';
            chart1.options.scales.y.title.text = 'Opening [mm]';
            chart1.options.scales.y.min = 0;
            chart1.options.scales.y.max = 80;
            chart2.options.plugins.title.text = 'Control Signal 6';
            chart2.data.datasets[0].label = 'u(t)';
            chart2.data.datasets[1].label = '';
            chart2.options.scales.y.title.text = 'Duty Cycle [%]';   
            chart2.options.scales.y.min = -100;
            chart2.options.scales.y.max = 100;       
        }
    }
    if(graphtype.value == 'B')
    {
        gdlselect.style.display = 'none';
        allLayout();

        chart1.options.plugins.title.text = 'Joint 1 Value';
        chart1.data.datasets[0].label = 'θ₁(t)';
        chart1.data.datasets[1].label = 'θref₁(t)';
        chart1.options.scales.y.title.text = 'Angle [°]';
        chart1.options.scales.y.min = -170;
        chart1.options.scales.y.max = 170;

        chart2.options.plugins.title.text = 'Joint 2 Value';
        chart2.data.datasets[0].label = 'θ₂(t)';
        chart2.data.datasets[1].label = 'θref₂(t)';
        chart2.options.scales.y.title.text = 'Angle [°]';
        chart2.options.scales.y.min = -90;
        chart2.options.scales.y.max = 30;

        chart3.options.plugins.title.text = 'Joint 3 Value';
        chart3.data.datasets[0].label = 'θ₃(t)';
        chart3.data.datasets[1].label = 'θref₃(t)';
        chart3.options.scales.y.title.text = 'Angle [°]';
        chart3.options.scales.y.min = -135;
        chart3.options.scales.y.max = 0;

        chart4.options.plugins.title.text = 'Joint 4 Value';
        chart4.data.datasets[0].label = 'θ₄(t)';
        chart4.data.datasets[1].label = 'θref₄(t)';
        chart4.options.scales.y.title.text = 'Angle [°]';
        chart4.options.scales.y.min = -135;
        chart4.options.scales.y.max = 135;

        chart5.options.plugins.title.text = 'Joint 5 Value';
        chart5.data.datasets[0].label = 'θ₅(t)';
        chart5.data.datasets[1].label = 'θref₅(t)';
        chart5.options.scales.y.title.text = 'Angle [°]';
        chart5.options.scales.y.min = -170;
        chart5.options.scales.y.max = 170;

        chart6.options.plugins.title.text = 'Grip Opening';
        chart6.data.datasets[0].label = 'd(t)';
        chart6.data.datasets[1].label = 'dref(t)';
        chart6.options.scales.y.title.text = 'Opening [mm]';
        chart6.options.scales.y.min = 0;
        chart6.options.scales.y.max = 80;
    } 
    if(graphtype.value == 'C')
    {
        gdlselect.style.display = 'none';
        allLayout();

        chart1.options.plugins.title.text = 'Control Signal 1';
        chart1.data.datasets[0].label = 'u₁(t)';
        chart1.data.datasets[1].label = '';
        chart1.options.scales.y.title.text = 'Duty Cycle [%]';
        chart1.options.scales.y.min = -100;
        chart1.options.scales.y.max = 100;

        chart2.options.plugins.title.text = 'Control Signal 2';
        chart2.data.datasets[0].label = 'u₂(t)';
        chart2.data.datasets[1].label = '';
        chart2.options.scales.y.title.text = 'Duty Cycle [%]';
        chart2.options.scales.y.min = -100;
        chart2.options.scales.y.max = 100;

        chart3.options.plugins.title.text = 'Control Signal 3';
        chart3.data.datasets[0].label = 'u₃(t)';
        chart3.data.datasets[1].label = '';
        chart3.options.scales.y.title.text = 'Duty Cycle [%]';
        chart3.options.scales.y.min = -100;
        chart3.options.scales.y.max = 100;

        chart4.options.plugins.title.text = 'Control Signal 4';
        chart4.data.datasets[0].label = 'u₄(t)';
        chart4.data.datasets[1].label = '';
        chart4.options.scales.y.title.text = 'Duty Cycle [%]';
        chart4.options.scales.y.min = -100;
        chart4.options.scales.y.max = 100;

        chart5.options.plugins.title.text = 'Control Signal 5';
        chart5.data.datasets[0].label = 'u₅(t)';
        chart5.data.datasets[1].label = '';
        chart5.options.scales.y.title.text = 'Duty Cycle [%]';
        chart5.options.scales.y.min = -100;
        chart5.options.scales.y.max = 100;

        chart6.options.plugins.title.text = 'Control Signal 6';
        chart6.data.datasets[0].label = 'u(t)';
        chart6.data.datasets[1].label = '';
        chart6.options.scales.y.title.text = 'Duty Cycle [%]';
        chart6.options.scales.y.min = -100;
        chart6.options.scales.y.max = 100;
    } 
    if(graphtype.value == 'D')
    {
        gdlselect.style.display = 'flex';
        allLayout();
        chart1.options.plugins.title.text = 'Error Value';
        chart1.data.datasets[0].label = 'ε(t)';
        chart1.data.datasets[1].label = '';
        chart1.options.scales.y.title.text = 'Angle [°]';
        chart1.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart1.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart2.options.plugins.title.text = 'Error Integral';
        chart2.data.datasets[0].label = '∫ε(t)';
        chart2.data.datasets[1].label = '';
        chart2.options.scales.y.title.text = 'Angle [°]';
        chart2.options.scales.y.min = chart2.options.scales.y.suggestedMin;
        chart2.options.scales.y.max = chart2.options.scales.y.suggestedMax;
        
        chart3.options.plugins.title.text = 'Error Derivative';
        chart3.data.datasets[0].label = 'dε(t)';
        chart3.data.datasets[1].label = '';
        chart3.options.scales.y.title.text = 'Angle [°]';
        chart3.options.scales.y.min = chart3.options.scales.y.suggestedMin;
        chart3.options.scales.y.max = chart3.options.scales.y.suggestedMax;

        chart4.options.plugins.title.text = 'Proportional Control Signal';
        chart4.data.datasets[0].label = 'up(t)';
        chart4.data.datasets[1].label = '';
        chart4.options.scales.y.title.text = 'Duty Cicle [%]';
        chart4.options.scales.y.min = chart4.options.scales.y.suggestedMin;
        chart4.options.scales.y.max = chart4.options.scales.y.suggestedMax;

        chart5.options.plugins.title.text = 'Integral Control Signal';
        chart5.data.datasets[0].label = 'ui(t)';
        chart5.data.datasets[1].label = '';
        chart5.options.scales.y.title.text = 'Duty Cicle [%]';
        chart5.options.scales.y.min = chart5.options.scales.y.suggestedMin;
        chart5.options.scales.y.max = chart5.options.scales.y.suggestedMax;
        
        chart6.options.plugins.title.text = 'Derivative Control Signal';
        chart6.data.datasets[0].label = 'ud(t)';
        chart6.data.datasets[1].label = '';
        chart6.options.scales.y.title.text = 'Duty Cicle [%]';
        chart6.options.scales.y.min = chart6.options.scales.y.suggestedMin;
        chart6.options.scales.y.max = chart6.options.scales.y.suggestedMax; 
    }
    if(graphtype.value == 'E')
    {
        gdlselect.style.display = 'none';
        allLayout();

        chart1.options.plugins.title.text = 'X Coordinate';
        chart1.data.datasets[0].label = 'x(t)';
        chart1.data.datasets[1].label = '';
        chart1.options.scales.y.title.text = 'X [mm]';
        chart1.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart1.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart2.options.plugins.title.text = 'Y Coordinate';
        chart2.data.datasets[0].label = 'y(t)';
        chart2.data.datasets[1].label = '';
        chart2.options.scales.y.title.text = 'Y [mm]';
        chart2.options.scales.y.min = chart2.options.scales.y.suggestedMin;
        chart2.options.scales.y.max = chart2.options.scales.y.suggestedMax;

        chart3.options.plugins.title.text = 'Z Coordinate';
        chart3.data.datasets[0].label = 'z(t)';
        chart3.data.datasets[1].label = '';
        chart3.options.scales.y.title.text = 'Z [mm]';
        chart3.options.scales.y.min = chart3.options.scales.y.suggestedMin;
        chart3.options.scales.y.max = chart3.options.scales.y.suggestedMax;

        chart4.options.plugins.title.text = 'A - Euler Angle';
        chart4.data.datasets[0].label = 'a(t)';
        chart4.data.datasets[1].label = '';
        chart4.options.scales.y.title.text = 'A [°]';
        chart4.options.scales.y.min = chart4.options.scales.y.suggestedMin;
        chart4.options.scales.y.max = chart4.options.scales.y.suggestedMax;

        chart5.options.plugins.title.text = 'E - Euler Angle';
        chart5.data.datasets[0].label = 'e(t)';
        chart5.data.datasets[1].label = '';
        chart5.options.scales.y.title.text = 'E [°]';
        chart5.options.scales.y.min = chart5.options.scales.y.suggestedMin;
        chart5.options.scales.y.max = chart5.options.scales.y.suggestedMax;

        chart6.options.plugins.title.text = 'R - Euler Angle';
        chart6.data.datasets[0].label = 'r(t)';
        chart6.data.datasets[1].label = '';
        chart6.options.scales.y.title.text = 'R [°]';
        chart6.options.scales.y.min = chart6.options.scales.y.suggestedMin;
        chart6.options.scales.y.max = chart6.options.scales.y.suggestedMax;
    }
    if(graphtype.value == 'F')
    {
        gdlselect.style.display = 'none';
        allLayout();

        chart1.options.plugins.title.text = 'Motor Current 1';
        chart1.data.datasets[0].label = 'I₁(t)';
        chart1.data.datasets[1].label = '';
        chart1.options.scales.y.title.text = 'Current [A]';
        chart1.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart1.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart2.options.plugins.title.text = 'Motor Current 2';
        chart2.data.datasets[0].label = 'I₂(t)';
        chart2.data.datasets[1].label = '';
        chart2.options.scales.y.title.text = 'Current [A]';
        chart2.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart2.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart3.options.plugins.title.text = 'Motor Current 3';
        chart3.data.datasets[0].label = 'I₃(t)';
        chart3.data.datasets[1].label = '';
        chart3.options.scales.y.title.text = 'Current [A]';
        chart3.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart3.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart4.options.plugins.title.text = 'Motor Current 4';
        chart4.data.datasets[0].label = 'I₄(t)';
        chart4.data.datasets[1].label = '';
        chart4.options.scales.y.title.text = 'Current [A]';
        chart4.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart4.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart5.options.plugins.title.text = 'Motor Current 5';
        chart5.data.datasets[0].label = 'I₅(t)';
        chart5.data.datasets[1].label = '';
        chart5.options.scales.y.title.text = 'Current [A]';
        chart5.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart5.options.scales.y.max = chart1.options.scales.y.suggestedMax;

        chart6.options.plugins.title.text = 'Motor Current 6';
        chart6.data.datasets[0].label = 'I₆(t)';
        chart6.data.datasets[1].label = '';
        chart6.options.scales.y.title.text = 'Current [A]';
        chart6.options.scales.y.min = chart1.options.scales.y.suggestedMin;
        chart6.options.scales.y.max = chart1.options.scales.y.suggestedMax;
    }
    clearGraphics();
};

function clearGraphics() {
    chartData.time.length = 0;
    for(let i=0; i<6; i++) 
    {
        chartData.data[i].length = 0;
        chartData.ref[i].length = 0;
    }
    chart1.update();
    chart2.update();
    chart3.update();
    chart4.update();
    chart5.update();
    chart6.update();
};

graphtype.addEventListener('change', () => {
    updateView();
});

gdlselect.addEventListener('change', () => {
    updateView();
});

//Envio da velocidade ao manipulador?
const spdovr = document.getElementById('spdovr');
spdovr.addEventListener('input', () => {
    sendMqtt('request', 'D'+spdovr.value.toString()+',');
});

//Envio de PWM em malha aberta:
sendref.addEventListener('click', () => {
    if(looptype.value == 'O')
    {
        sendMqtt('request', 'O'+jointselect.value[1]+','+refval.value.toString()+',');
    }else
    {
        sendMqtt('request', 'G'+jointselect.value[1]+','+refval.value.toString()+',');
    }
});

const botgo = document.getElementById('botgo');

//Envio de target para movimentação:
botgo.addEventListener('click', () => {
    if(movetype.value == 'MJ')
    {
        sendMqtt('request', 'J'+tg1.value.toString()+','+tg2.value.toString()+','+tg3.value.toString()+','+tg4.value.toString()+','+tg5.value.toString()+',');
    }
});

const bothome = document.getElementById('bothome');

//Envio de target para movimentação:
bothome.addEventListener('click', () => {
    startButton.click()
    sendMqtt('request', 'J0,0,-5,0,0,');
});
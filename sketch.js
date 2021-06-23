/*******************************/
/* Control and Simulation Code */
/*******************************/

// PID controller
class PID {
  constructor(Kp, Ki, Kd, set_point) {
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
    this.set_point = set_point;
    this.int_term = 0
    this.derivative_term = 0
    this.last_error = null
  }

  get_control(measurement, dt){
     var error = this.set_point - measurement;
     this.int_term += error*this.Ki*dt;
     if (this.last_error != null){
       this.derivative_term = (error-this.last_error)/dt*this.Kd
     }
     
     this.last_error = error
     return this.Kp * error + this.int_term + this.derivative_term
  }       
}

// drone simulation
let g = -9.81
let p_desired = 50

// system update
function next_state(a, state, dt){
    var aeff = a + g;
    var position = state[0];
    var velocity = state[1];
    //integration by velocity verlet algorithm
    position += velocity*dt + 0.5*aeff*dt*dt;
    velocity += aeff*dt;
    return [position,velocity];
}

function simulate_drone(controller, num_steps = 1000){
    var dt = 0.02;
    var state = [0,0];
    var pos_list=[state[0]];
    var time_list = [0];
    var a = 0;
    for (let i = 0; i < num_steps; i++) {
        a = controller.get_control(state[0], dt);
        // upwards acceleration is positive (>0)
        // and limited by power supply (<100)
        a = clip(a, 0, 100);
        state = next_state(a, state, dt);
        pos_list.push(state[0]);
        time_list.push((i+1)*dt);
    }
    return [time_list, pos_list];
}
  
function clip(x, xmin, xmax) // simple helper
{
  if (x<xmin)
    return xmin;
  if (x>xmax)
    return xmax;
  return x;
}

/*******************************/
/* Visualization code **********/
/*******************************/

let states; // 2d array containing 1d time array and 1d temperature array
let controller;

let slider_p;
let slider_i;
let slider_d;

// layout for plotly plot
let layout = {
  showlegend: false,
  font: {
    size: 18
  },
  title: '',
  xaxis: {
    title: 'Time',
  },
  yaxis: {
    title: 'position',
    showline: false,
    range: [-75,75]
  }
};


function re_run_sim() {
  controller = new PID(slider_p.value()/100.0,
                       slider_i.value()/100.0,
                       slider_d.value()/100.0,
                       p_desired);
  states = simulate_drone(controller);
  
  
  var trace_T = {
    x: states[0],
    y: states[1],
    name : 'p',
    type: 'line'
  };
  
  var trace_T_desired = {
    x: states[0],
    y: Array(states[0].length).fill(p_desired),
    name : 'p_desired',
    type: 'line'
  };
  
  super_group.position(0.25*windowWidth,0);

  slider_p_text.html('  Kp = '+my_display_float(slider_p.value()/100.0,2)+ ' ');
  slider_i_text.html('  Ki = '+my_display_float(slider_i.value()/100.0,2)+ ' ');
  slider_d_text.html('  Kd = '+my_display_float(slider_d.value()/100.0,2));
    
  p = Plotly.newPlot('myDiv', [trace_T_desired,trace_T], layout,{displayModeBar: false});
}


function setup(){

  super_group = createDiv('');
  
  super_group.style('width', '100%')
  
  group_p = createDiv('');
  group_p.parent(super_group);
  slider_p = createSlider(0,1000, 0);
  slider_p.style('width', '50%');
  slider_p.input(re_run_sim);
  slider_p.parent(group_p);
  slider_p_text = createSpan();
  slider_p_text.parent(group_p);
  slider_p_text.style('font-size', '18px');
  
  group_i = createDiv('');
  group_i.parent(super_group);
  slider_i = createSlider(0,50, 0);
  slider_i.style('width', '50%');
  slider_i.input(re_run_sim);
  slider_i.parent(group_i);
  slider_i_text = createSpan();
  slider_i_text.parent(group_i);
  slider_i_text.style('font-size', '18px');
  
  group_d = createDiv('');
  group_d.parent(super_group);
  slider_d = createSlider(-300,300, 0);  
  slider_d.style('width', '50%');
  slider_d.input(re_run_sim);
  slider_d.parent(group_d);
  slider_d_text = createSpan();
  slider_d_text.parent(group_d);
  slider_d_text.style('font-size', '18px');
  
  
  frameRate(30);
  
  
  re_run_sim();
}

function my_display_float(x,n) { // simple helper
  return Number.parseFloat(x).toFixed(n);
}


function draw() {    
  
}

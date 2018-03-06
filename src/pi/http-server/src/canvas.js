/**
 * canvas.js: sets up the html5 canvas that we will render to. also manages
 * the animation loop
 * 
 * Author: Alex Haggart
 */
import {mat4,vec4} from 'gl-matrix';
import {Link} from './link.js';

import vsSource from './basic-vertex-shader.vs';
import fsSource from './basic-frag-shader.fs';

//
// Initialize a shader program, so WebGL knows how to draw our data
//
function initShaderProgram(gl, vsSource, fsSource) {
  const vertexShader = loadShader(gl, gl.VERTEX_SHADER, vsSource);
  const fragmentShader = loadShader(gl, gl.FRAGMENT_SHADER, fsSource);

  // Create the shader program

  const shaderProgram = gl.createProgram();
  gl.attachShader(shaderProgram, vertexShader);
  gl.attachShader(shaderProgram, fragmentShader);
  gl.linkProgram(shaderProgram);

  // If creating the shader program failed, alert

  if (!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
    console.error('Unable to initialize the shader program: ' + gl.getProgramInfoLog(shaderProgram));
    return null;
  }

  return shaderProgram;
}

//
// creates a shader of the given type, uploads the source and
// compiles it.
//
function loadShader(gl, type, source) {
  const shader = gl.createShader(type);

  // Send the source to the shader object
  gl.shaderSource(shader, source);

  // Compile the shader program
  gl.compileShader(shader);

  // See if it compiled successfully
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    console.error('An error occurred compiling the shaders: ' + gl.getShaderInfoLog(shader));
    gl.deleteShader(shader);
    return null;
  }

  return shader;
}

function draw(gl,programInfo,drawList,deltaTime){
  gl.clearColor(0.0, 0.0, 0.0, 1.0);  // Clear to black, fully opaque
  gl.clearDepth(1.0);                 // Clear everything
  gl.enable(gl.DEPTH_TEST);           // Enable depth testing
  gl.depthFunc(gl.LEQUAL);            // Near things obscure far things

  // Clear the canvas before we start drawing on it.

  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  //TODO: this should probably go somewhere else, since it is static
  // Create a perspective matrix, a special matrix that is
  // used to simulate the distortion of perspective in a camera.
  // Our field of view is 45 degrees, with a width/height
  // ratio that matches the display size of the canvas
  // and we only want to see objects between 0.1 units
  // and 100 units away from the camera.

  const fieldOfView = 45 * Math.PI / 180;   // in radians
  const aspect = gl.canvas.clientWidth / gl.canvas.clientHeight;
  const zNear = 0.1;
  const zFar = 100.0;
  const projectionMatrix = mat4.create();

  // note: glmatrix.js always has the first argument
  // as the destination to receive the result.
  mat4.perspective(projectionMatrix,
                   fieldOfView,
                   aspect,
                   zNear,
                   zFar);

  // Tell WebGL to use our program when drawing
  gl.useProgram(programInfo.program);

  // Set the shader uniforms

  gl.uniformMatrix4fv(
      programInfo.uniformLocations.projectionMatrix,
      false,
      projectionMatrix);

  drawList.forEach((obj)=>obj.draw(gl));
}

//
// start here
//
function main() {
  const canvas = document.querySelector("#glCanvas");
  // Initialize the GL context
  const gl = canvas.getContext("webgl");

  // Only continue if WebGL is available and working
  if (!gl) {
    console.error("Unable to initialize WebGL. Your browser or machine may not support it.");
    return;
  }

  initKeys();

  // Set clear color to black, fully opaque
  gl.clearColor(0.0, 0.0, 0.0, 1.0);
  // Clear the color buffer with specified clear color
  gl.clear(gl.COLOR_BUFFER_BIT);

  // const vsSource = require('./basic-vertex-shader.vs');
  
  // const fsSource = require('./basic-frag-shader.fs');

  const shaderProgram = initShaderProgram(gl, vsSource, fsSource);

  const programInfo = {
    program: shaderProgram,
    attribLocations: {
      vertexPosition: gl.getAttribLocation(shaderProgram, 'aVertexPosition'),
      vertexColor: gl.getAttribLocation(shaderProgram,'aVertexColor'),
    },
    uniformLocations: {
      projectionMatrix: gl.getUniformLocation(shaderProgram, 'uProjectionMatrix'),
      modelViewMatrix: gl.getUniformLocation(shaderProgram, 'uModelViewMatrix'),
    },
  };

  const base_transform = mat4.create();
  mat4.translate(base_transform,base_transform,[-5,0.0,-20.0])

  const base_link = {
    getTransform:()=>base_transform,
    addChild:()=>{},
  };

  const base_joint0 = {angle:0,axis:[0,1,0]};
  const base0 = new Link(0,0,base_joint0,base_link);
  base0.build(gl,shaderProgram);

  const joint0 = {angle:3.1415/4,axis:[0,0,1]};
  const link0 = new Link(4,1,joint0,base0);
  link0.build(gl,shaderProgram);

  const joint1 = {angle:-3.1415/4,axis:[0,0,1]};
  const link1 = new Link(4,1,joint1,link0);
  link1.build(gl,shaderProgram);

  const joint2 = {angle:-3.1415/4,axis:[0,0,1]};
  const link2 = new Link(4,1,joint2,link1);
  link2.build(gl,shaderProgram);

  const knuckle00 = {angle:-3.1415/3,axis:[0,0,1]};
  const finger00 = new Link(1,0.5,knuckle00,link2);
  finger00.build(gl,shaderProgram);

  const knuckle01 = {angle:3.1415/3,axis:[0,0,1]};
  const finger01 = new Link(1,0.5,knuckle01,finger00);
  finger01.build(gl,shaderProgram);

  const knuckle10 = {angle:3.1415/3,axis:[0,0,1]};
  const finger10 = new Link(1,0.5,knuckle10,link2);
  finger10.build(gl,shaderProgram);

  const knuckle11 = {angle:-3.1415/3,axis:[0,0,1]};
  const finger11 = new Link(1,0.5,knuckle11,finger10);
  finger11.build(gl,shaderProgram);


  const drawList = [
    base0,
    link0,link1,link2,
    finger00,finger01,
    finger10,finger11,
  ];

  var then = 0;
  var totalTime = 0;
  // Draw the scene repeatedly
  function render(now) {
    now *= 0.001;  // convert to seconds
    const deltaTime = now - then;
    then = now;

    draw(gl,programInfo,drawList,deltaTime);
    totalTime += deltaTime;

    if(keys.q){
      link0.move(deltaTime);
    } else if(keys.a){
      link0.move(-deltaTime);
    }

    if(keys.w){
      link1.move(deltaTime);
    } else if(keys.s){
      link1.move(-deltaTime);
    }

    if(keys.e){
      link2.move(deltaTime);
    } else if(keys.d){
      link2.move(-deltaTime);
    }

    if(keys.z){
      base0.move(-deltaTime);
    } else if(keys.c){
      base0.move(deltaTime);
    }

    if(keys[' ']&&knuckle00.angle < -3.1415/5){
      finger00.move(deltaTime);
      finger10.move(-deltaTime);
    } else if(!keys[' ']&&knuckle00.angle > -3.1415/3){
      finger00.move(-deltaTime);
      finger10.move(+deltaTime);
    }

    requestAnimationFrame(render);
  }
  requestAnimationFrame(render);
}

var keys;
function initKeys(){
  keys = {
    'q':false,
    'w':false,
    'e':false,
    'a':false,
    's':false,
    'd':false,
    'z':false,
    'c':false,
    ' ':false,
  }
  window.addEventListener('keydown',keyDown,false);
  window.addEventListener('keyup',keyUp,false);
}

function keyDown(e){
  if(e.key in keys){
    keys[e.key] = true;
  }
}

function keyUp(e){
  if(e.key in keys){
    keys[e.key] = false;
  }
}

main();

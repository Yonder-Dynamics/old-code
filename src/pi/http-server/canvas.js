var squareRotation = 0.0;

function createAndBindBuffer(gl,type,data,usage){
  const buffer = gl.createBuffer();
  gl.bindBuffer(type,buffer);
  gl.bufferData(type,data,usage);
  return buffer;
}

function enableVertexFloatArrayBuffer(gl,buffer,position,indexSize){
  const numComponents = indexSize;  // pull out 3 values per iteration
  const type = gl.FLOAT;    // the data in the buffer is 32bit floats
  const normalize = false;  // don't normalize
  const stride = 0;         // how many bytes to get from one set of values to the next
                            // 0 = use type and numComponents above
  const offset = 0;         // how many bytes inside the buffer to start from
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.vertexAttribPointer(
      position,
      numComponents,
      type,
      normalize,
      stride,
      offset);
  gl.enableVertexAttribArray(
      position);
}

class Cubic{
  constructor(x,y,z){
    this.x = x;
    this.y = y;
    this.z = z;

    //rgba
    this.color = [0.2,0.2,0.9,1.0];

    this.buffers = {};
    this.positions = {};
    this.uniforms = {};
  }

  build(gl,shaderProgram){
    //create vertex array
    const vertices = [
      -this.x/2,  -this.y/2, -this.z/2, //front face
       this.x/2,  -this.y/2, -this.z/2,
       this.x/2,   this.y/2, -this.z/2,
      -this.x/2,   this.y/2, -this.z/2,

      -this.x/2,  -this.y/2,  this.z/2, //back face
       this.x/2,  -this.y/2,  this.z/2,
       this.x/2,   this.y/2,  this.z/2,
      -this.x/2,   this.y/2,  this.z/2,
    ];

    // this.buffers.vertexBuffer = gl.createBuffer();
    // gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
    // gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(vertices),gl.STATIC_DRAW);
    this.buffers.vertexBuffer = createAndBindBuffer(gl,gl.ARRAY_BUFFER,new Float32Array(vertices),gl.STATIC_DRAW);

    const colors = [
      0.2,  0.2,  0.9,  1.0,
      0.2,  0.2,  0.9,  1.0,
      0.2,  0.2,  0.9,  1.0,
      0.2,  0.2,  0.9,  1.0,

      0.2,  0.2,  0.9,  1.0,
      0.2,  0.2,  0.9,  1.0,
      0.2,  0.2,  0.9,  1.0,
      0.2,  0.2,  0.9,  1.0,
    ];

    this.buffers.colorBuffer = createAndBindBuffer(gl,gl.ARRAY_BUFFER,new Float32Array(colors),gl.STATIC_DRAW);

    const indices = [
      0,1,1,2,2,3,3,0, //front face
      4,5,5,6,6,7,7,4, //back face
      0,4,1,5,2,6,3,7, //connections
    ];

    this.buffers.indexBuffer = createAndBindBuffer(gl,gl.ELEMENT_ARRAY_BUFFER,new Uint16Array(indices),gl.STATIC_DRAW);

    this.positions.vertexBuffer = gl.getAttribLocation(shaderProgram, 'aVertexPosition');
    this.positions.colorBuffer  = gl.getAttribLocation(shaderProgram, 'aVertexColor');

    return this;
  }

  draw(gl){

    enableVertexFloatArrayBuffer(gl,this.buffers.vertexBuffer,this.positions.vertexBuffer,3);
    enableVertexFloatArrayBuffer(gl,this.buffers.colorBuffer,this.positions.colorBuffer,4);

    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.buffers.indexBuffer);

    //draw everything
    {
      const offset = 0;
      const vertexCount = 24;
      const type = gl.UNSIGNED_SHORT;
      gl.drawElements(gl.LINES,vertexCount,type,offset);
    }
  }
}

class Link{
  constructor(length,width,joint,parent){
    this.wireframe = new Cubic(length,width,width);
    this.uniforms = {};
    this.parent = parent;
    this.children = [];
    this.parent.addChild(this);
    this.joint = joint;
    this.length = length;

    this.buildTransforms();
    // const origin = vec4.create();
    // vec4.set(origin,0,0,0,1);
    // vec4.transformMat4(origin,origin,this.connector);

    // console.log(this.connector);
    
  }

  buildTransforms(){
    this.transform = mat4.create();
    // Now move the drawing position a bit to where we want to
    // start drawing the square.
    mat4.fromRotation(this.transform,this.joint.angle,this.joint.axis);
    const translation = mat4.create();
    mat4.fromTranslation(translation,[this.length/2, 0.0, 0.0]); //position one end of segment 
    mat4.mul(this.transform,this.transform,translation);

    const world_space = mat4.create();
    mat4.copy(world_space,this.parent.getTransform());

    this.connector = mat4.create();
    mat4.fromTranslation(this.connector,[this.length/2,0,0]);
    mat4.mul(world_space,world_space,this.transform);
    mat4.mul(this.connector,world_space,this.connector);
  }

  build(gl,shaderProgram){
    this.wireframe.build(gl,shaderProgram);


    // mat4.rotate(modelViewMatrix,  // destination matrix
    //           modelViewMatrix,  // matrix to rotate
    //           squareRotation * 0.7,   // amount to rotate in radians
    //           [0, 1, 0]);       // axis to rotate around
    this.uniforms.transform     = gl.getUniformLocation(shaderProgram,'uModelViewMatrix');

  }

  addChild(child){
    this.children.push(child);
  }

  update(angle=this.joint.angle){
    this.joint.angle = angle;
    this.buildTransforms();
    this.children.forEach((child)=>child.update());
  }

  draw(gl){
    const world_space = mat4.create();
    mat4.mul(world_space,this.parent.getTransform(),this.transform);


    gl.uniformMatrix4fv(
      this.uniforms.transform,
      false,
      world_space);

    this.wireframe.draw(gl);
  }

  getTransform(){
    return this.connector;
  }
}

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
    alert('Unable to initialize the shader program: ' + gl.getProgramInfoLog(shaderProgram));
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
    alert('An error occurred compiling the shaders: ' + gl.getShaderInfoLog(shader));
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
  squareRotation += deltaTime;
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
    alert("Unable to initialize WebGL. Your browser or machine may not support it.");
    return;
  }

  // Set clear color to black, fully opaque
  gl.clearColor(0.0, 0.0, 0.0, 1.0);
  // Clear the color buffer with specified clear color
  gl.clear(gl.COLOR_BUFFER_BIT);

  const vsSource = `
    attribute vec4 aVertexPosition;
    attribute vec4 aVertexColor;

    uniform mat4 uModelViewMatrix;
    uniform mat4 uProjectionMatrix;

    varying lowp vec4 vColor;

    void main() {
      gl_Position = uProjectionMatrix * uModelViewMatrix * aVertexPosition;
      vColor = aVertexColor;
    }
  `;

  const fsSource = `
    varying lowp vec4 vColor;
    void main() {
      gl_FragColor = vColor;
    }
  `;

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

  const joint0 = {angle:3.1415/4,axis:[0,0,1]};
  const link0 = new Link(4,1,joint0,base_link);
  link0.build(gl,shaderProgram);

  const joint1 = {angle:-3.1415/4,axis:[0,0,1]};
  const link1 = new Link(4,1,joint1,link0);
  link1.build(gl,shaderProgram);

  const joint2 = {angle:-3.1415/4,axis:[0,0,1]};
  const link2 = new Link(4,1,joint2,link1);
  link2.build(gl,shaderProgram);

  const knuckle00 = {angle:-3.1415/4,axis:[0,0,1]};
  const finger00 = new Link(1,0.5,knuckle00,link2);
  finger00.build(gl,shaderProgram);

  const knuckle01 = {angle:3.1415/3,axis:[0,0,1]};
  const finger01 = new Link(1,0.5,knuckle01,finger00);
  finger01.build(gl,shaderProgram);

  const knuckle10 = {angle:3.1415/4,axis:[0,0,1]};
  const finger10 = new Link(1,0.5,knuckle10,link2);
  finger10.build(gl,shaderProgram);

  const knuckle11 = {angle:-3.1415/3,axis:[0,0,1]};
  const finger11 = new Link(1,0.5,knuckle11,finger10);
  finger11.build(gl,shaderProgram);


  const drawList = [
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
    link0.update(totalTime);

    requestAnimationFrame(render);
  }
  requestAnimationFrame(render);
}

main();

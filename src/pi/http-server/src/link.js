const mat = require('gl-matrix');
import {Cubic} from './cubic.js';

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
    this.transform = mat.mat4.create();
    // Now move the drawing position a bit to where we want to
    // start drawing the square.
    mat.mat4.fromRotation(this.transform,this.joint.angle,this.joint.axis);
    const translation = mat.mat4.create();
    mat.mat4.fromTranslation(translation,[this.length/2, 0.0, 0.0]); //position one end of segment 
    mat.mat4.mul(this.transform,this.transform,translation);

    const world_space = mat.mat4.create();
    mat.mat4.copy(world_space,this.parent.getTransform());

    this.connector = mat.mat4.create();
    mat.mat4.fromTranslation(this.connector,[this.length/2,0,0]);
    mat.mat4.mul(world_space,world_space,this.transform);
    mat.mat4.mul(this.connector,world_space,this.connector);
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

  move(angle){
    this.update(angle+this.joint.angle);
  }

  draw(gl){
    const world_space = mat.mat4.create();
    mat.mat4.mul(world_space,this.parent.getTransform(),this.transform);


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

export {Link};

// export var __useDefault = true;


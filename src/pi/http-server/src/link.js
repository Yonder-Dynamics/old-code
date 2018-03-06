/**
 * link.js: basic transform link. will probably be further generalized and
 * subclassed in the future to allow more options for rendering
 * 
 * Author: Alex Haggart
 */
import {mat4} from 'gl-matrix';
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

export {Link};

// export var __useDefault = true;


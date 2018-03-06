/**
 * post_button.js: proof-of-concept code for data transfer via javascript
 * 
 * Author: Alex Haggart, some guy on stack overflow
 */
function postFunction(){
  var url = "http://100.80.225.75:8000";
  var method = "POST";
  var postData = "Some data";

  var shouldBeAsync = true;

  var request = new XMLHttpRequest();

  request.onload = function () {
     var status = request.status; 
     var data = request.responseText; 
  }

  request.open(method, url, shouldBeAsync);

  request.setRequestHeader("Content-Type", "application/json;charset=UTF-8");

  // Actually sends the request to the server.
  request.send(postData);
}

document.getElementById("post_button").onclick = postFunction;

function postFunction(){
  var url = "http://100.80.225.75:8000";
  var method = "POST";
  var postData = "Some data";

  // You REALLY want shouldBeAsync = true.
  // Otherwise, it'll block ALL execution waiting for server response.
  var shouldBeAsync = true;

  var request = new XMLHttpRequest();

  // Before we send anything, we first have to say what we will do when the
  // server responds. This seems backwards (say how we'll respond before we send
  // the request? huh?), but that's how Javascript works.
  // This function attached to the XMLHttpRequest "onload" property specifies how
  // the HTTP response will be handled. 
  request.onload = function () {

     // Because of javascript's fabulous closure concept, the XMLHttpRequest "request"
     // object declared above is available in this function even though this function
     // executes long after the request is sent and long after this function is
     // instantiated. This fact is CRUCIAL to the workings of XHR in ordinary
     // applications.

     // You can get all kinds of information about the HTTP response.
     var status = request.status; // HTTP response status, e.g., 200 for "200 OK"
     var data = request.responseText; // Returned data, e.g., an HTML document.
  }

  request.open(method, url, shouldBeAsync);

  request.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
  // Or... request.setRequestHeader("Content-Type", "text/plain;charset=UTF-8");
  // Or... whatever

  // Actually sends the request to the server.
  request.send(postData);
}

document.getElementById("post_button").onclick = postFunction;

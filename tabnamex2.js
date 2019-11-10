/*
Copyright (c) 2010 Aza Raskin
http://azarask.in

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.
*/


(function(){

var TIMER = null;
var HAS_SWITCHED = false;

// Events
window.onblur = function(){
  TIMER = setTimeout(changeItUp, 5000);
}

window.onfocus = function(){
  if(TIMER) clearTimeout(TIMER);
}

// Utils
function setTitle(text){ document.title = text; }

// This favicon object rewritten from:
// Favicon.js - Change favicon dynamically [http://ajaxify.com/run/favicon].
// Copyright (c) 2008 Michael Mahemoff. Icon updates only work in Firefox and Opera.

favicon = {
  docHead: document.getElementsByTagName("head")[0],
  set: function(url){
    this.addLink(url);
  },

  addLink: function(iconURL) {
    var link = document.createElement("link");
    link.type = "image/x-icon";
    link.rel = "shortcut icon";
    link.href = iconURL;
    this.removeLinkIfExists();
    this.docHead.appendChild(link);
  },

  removeLinkIfExists: function() {
    var links = this.docHead.getElementsByTagName("link");
    for (var i=0; i<links.length; i++) {
      var link = links[i];
      if (link.type=="image/x-icon" && link.rel=="shortcut icon") {
        this.docHead.removeChild(link);
        return; // Assuming only one match at most.
      }
    }
  },

  get: function() {
    var links = this.docHead.getElementsByTagName("link");
    for (var i=0; i<links.length; i++) {
      var link = links[i];
      if (link.type=="image/x-icon" && link.rel=="shortcut icon") {
        return link.href;
      }
    }
  }
};


function createShield(){
  div = document.createElement("div");
  div.style.position = "fixed";
  div.style.top = 0;
  div.style.left = 0;
  div.style.backgroundColor = "white";
  div.style.width = "100%";
  div.style.height = "100%";
  div.style.textAlign = "center";
  document.body.style.overflow = "hidden";

  img = document.createElement("img");
  img.style.paddingTop = "15px";
  img.src = "http://img.skitch.com/20100524-b639xgwegpdej3cepch2387ene.png";

  var oldTitle = document.title;
  var oldFavicon = favicon.get() || "/favicon.ico";

  div.appendChild(img);
  document.body.appendChild(div);
  img.onclick = function(){
    div.parentNode.removeChild(div);
    document.body.style.overflow = "auto";
    setTitle(oldTitle);
    favicon.set(oldFavicon)
  }


}

function changeItUp(){
  if( HAS_SWITCHED == false ){
    createShield("https://mail.google.com");
    setTitle( "Gmail: Email from Google");
    favicon.set("https://mail.google.com/favicon.ico");
    HAS_SWITCHED = true;
  }
}


})();

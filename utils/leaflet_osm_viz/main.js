/**
 * This file is part of the LineCoverage-library.
 * The file contains javascript to obtain road network data using OpenStreetMap
 * Leaflet is used to create the user interace
 *
 * TODO:
 *
 * @author Saurav Agarwal
 * @contact sagarw10@uncc.edu
 * @contact agr.saurav1@gmail.com
 * Repository: https://github.com/UNCCharlotte-Robotics/LineCoverage-library
 *
 * Copyright (C) 2020--2022 University of North Carolina at Charlotte.
 * The LineCoverage-library is owned by the University of North Carolina at Charlotte and is protected by United States copyright laws and applicable international treaties and/or conventions.
 *
 * The LineCoverage-library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.
 *
 * SUPPORT AND MAINTENANCE: No support, installation, or training is provided.
 *
 * You should have received a copy of the GNU General Public License along with LineCoverage-library. If not, see <https://www.gnu.org/licenses/>.
 */

var map = L.map('map').setView([35.306, -80.734], 16);
var map_data_json;
var overpass_query_string = 'highway~"^(motorway|motorway_link|trunk|trunk_link|primary|secondary|tertiary|unclassified|residential)$"';

// L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=<access_token>', {
// 	maxZoom: 20,
// 	attribution: '<a href="https://github.com/UNCCharlotte-Robotics/LineCoverage-library">LineCoverage-library</a> | Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, ' +
// 	'Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
// 	id: 'mapbox/streets-v11',
// 	tileSize: 512,
// 	zoomOffset: -1
// }).addTo(map);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
	attribution: '<a href="https://github.com/UNCCharlotte-Robotics/LineCoverage-library">LineCoverage-library</a> | &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors | <a href="https://operations.osmfoundation.org/policies/tiles/">OpenStreetMap usage policy</a> | <a href="https://github.com/UNCCharlotte-Robotics/LineCoverage-library">LineCoverage-library</a>',
	maxZoom: 18,
	id: 'mapbox/streets-v11',
	tileSize: 512,
	zoomOffset: -1
}).addTo(map);

var drawnItems = L.featureGroup().addTo(map);
var graph = L.featureGroup().addTo(map);

const provider = new GeoSearch.OpenStreetMapProvider();
const search = new GeoSearch.GeoSearchControl({
	provider: new GeoSearch.OpenStreetMapProvider(),
});
map.addControl(search);

map.addLayer(drawnItems);
map.addLayer(graph);

var drawControl = new L.Control.Draw({
	position: 'topleft',
	draw: {
		polyline: false,
		polygon: {
			allowIntersection: false,
			showArea: true,
			metric: ['km'],
		},
		rectangle: {
			metric: ['km'],
		},
		marker: false,
		circle: false,
		circlemarker: false,
	},
	edit: {
		featureGroup: drawnItems,
		remove: true
	}
});
map.addControl(drawControl);

var download = L.Toolbar2.Action.extend({
	options: {
		toolbarIcon: {
			className: 'download-icon',
			html: '',
			tooltip: 'Download Files'
		},
	},
	addHooks: function () {
		saveAs(
			new Blob(
				[JSON.stringify(map_data_json)],{type: "application/json"}) , "map_data.json"
		);
	}
});

new L.Toolbar2.Control({
	position: 'topleft',
	actions: [download]
}).addTo(map);

L.control.layers({ }, {'Show Graph': graph, 'Show Drawing': drawnItems}, { position: 'topright', collapsed: false }).addTo(map);

// Truncate value based on number of decimals
var _round = function(num, len) {
	return Math.round(num*(Math.pow(10, len)))/(Math.pow(10, len));
};
// Helper method to format LatLng object (x.xxxxxx, y.yyyyyy)
var strLatLng = function(latlng) {
	return "("+_round(latlng.lat, 6)+", "+_round(latlng.lng, 6)+")";
};

// Generate popup content based on layer type
// - Returns HTML string, or null if unknown object
var getPopupContent = function(layer) {
	if (layer instanceof L.Polygon) {
		var latlngs = layer._defaultShape ? layer._defaultShape() : layer.getLatLngs(),
			area = L.GeometryUtil.geodesicArea(latlngs);
		return "Area: "+L.GeometryUtil.readableArea(area, true);
	}
	return null;
};

function buildOverpassApiUrl(poly_bounds, overpassQuery) {
	var bounds_str = 'poly: "';
	for(i = 0; i < poly_bounds.length; ++i) {
		if(i == 0) {
			bounds_str += poly_bounds[i].lat + ' ' + poly_bounds[i].lng;
		} else {
			bounds_str += ' ' + poly_bounds[i].lat + ' ' + poly_bounds[i].lng;
		}
	}
	bounds_str += '"';
	var wayQuery = 'way[' + overpassQuery + '](' + bounds_str + ');';
	var query = '?data=[out:json][timeout:15];(' + wayQuery + ');(._;>;);out skel;';
	var baseUrl = 'http://overpass-api.de/api/interpreter';
	var resultUrl = baseUrl + query;
	return resultUrl;
}

function GetOSMData(osm_url) {
	$.get(osm_url, function (osmDataAsJson) {
		map_data_json = osmDataAsJson;
		var resultAsGeojson = osmtogeojson(osmDataAsJson);
		var resultLayer = L.geoJson(resultAsGeojson, {
			style: function (feature) {
				return {color: "#2980b9", weight: 2};
			},
		});
		map.removeLayer(drawnItems);
		graph.clearLayers();
		graph.addLayer(resultLayer);
	});
}
// Object created - bind popup to layer, add to feature group
map.on(L.Draw.Event.CREATED, function(event) {
	var layer = event.layer;
	console.log(layer.getLatLngs()[0])
	var osm_url = buildOverpassApiUrl(layer.getLatLngs()[0], overpass_query_string);
	GetOSMData(osm_url);
	var content = getPopupContent(layer);
	if (content !== null) {
		layer.bindPopup(content);
	}
	drawnItems.clearLayers();
	drawnItems.addLayer(layer);
});

// Object(s) edited - update popups
map.on(L.Draw.Event.EDITED, function(event) {
	var layers = event.layers,
		content = null;
	layers.eachLayer(function(layer) {
		content = getPopupContent(layer);
		if (content !== null) {
			layer.setPopupContent(content);
		}
		var osm_url = buildOverpassApiUrl(layer.getLatLngs()[0], overpass_query_string);
		GetOSMData(osm_url);
	});
});

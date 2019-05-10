const express = require('express');
const routes = express.Router();
const controllers = require('./app/controllers');

routes.get('/users', (req, res) => {
  res.send('ok');
});

routes.get('/', controllers.LogController.store);

module.exports = routes;

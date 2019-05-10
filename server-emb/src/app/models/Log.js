const mongoose = require('mongoose');

const LogShema = new mongoose.Schema(
  {
    tmp: {
      type: Number
    }
  },
  {
    timestamps: true,
    collection: 'log'
  }
);

module.exports = mongoose.model('log', LogShema);

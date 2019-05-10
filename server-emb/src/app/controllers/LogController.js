class Log {
  async store(req, res) {
    res.send('ok');
  }
}

module.exports = new Log();

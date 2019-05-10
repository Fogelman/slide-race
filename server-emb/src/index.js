const app = require('./server');
require('dotenv').config();

app.express.listen(process.env.PORT || 3001);
